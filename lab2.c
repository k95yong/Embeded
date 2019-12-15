#include "includes.h"
#include <avr/io.h>   
#include <util/delay.h>

#define F_CPU   16000000UL   // CPU frequency = 16 Mhz
#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE            
#define  N_TASKS        5

#define   MBOX   0
#define   GVAR   1
#define   METHOD MBOX

#define DEBUG   1
#define DO 17
#define RE 43
#define MI 66
#define SOL 97
#define HDO 137
const unsigned char melody[4] = { DO,MI, SOL, HDO };

OS_STK TaskStk[N_TASKS][TASK_STK_SIZE];

OS_EVENT* MboxTemper;
OS_EVENT* MboxLight;
OS_EVENT* mySem;
OS_EVENT* Mutex;
OS_EVENT* Lqueue;
OS_EVENT* test;

/*
	시스템에서 가장 중요한 변수가 cur_state이다. 센서들에서 값을 받아와서 state값을 변화시키고
	그에 따라서 fnd display나 led, 부저등을 작동시킨다. 
	state는 0, 1, 2, 3, 4, 5의 값을 가지는데 순서대로 off, sleep, open, close, heat, cool의 의미를 나타낸다.
	각각의 의미는 다음과 같다.

	off : 시스템이 꺼져있는 상태
	sleep : 냉방과 난방시스템 가동이 필요하지 않은 상태로 수면에 적합한 온도를 유지한 상태(sleep mode)
	open : 조도가 낮아 커튼을 open한 상태(wake mode)
	close : 조도가 높아 커튼을 close한 상태(wake mode)
	heat : 난방시스템을 가동한 상태(sleep mode)
	cool : 냉방시스템을 가동한 상태(sleep mode)
*/
volatile INT8U FndNum, CDS, cur_state, slp_mode = 0, wake_mode = 0, pre_state = 0;
volatile INT8U mus = 0, mel_idx = 0;

void  TemperatureTask(void* data);
void  temperTask(void* data);
void  FndDisplayTask(void* data);
void  LightTask(void* data);
void  adcTask(void* data);
void* LqTbl[20];

ISR(INT4_vect) // 스위치 인터럽트 발생시 모드를 변경한다.(wake 모드 전환)
{
	offFnd();
	if (wake_mode == 0) // wake모드가 아닌 상태에서 스위치를 누르면 wake모드로 전환한다.
	{
		wake_mode = 1;
		slp_mode = 0;
	}
	else if (wake_mode == 1) {

		wake_mode = 0;
	}
	_delay_ms(10);
}

ISR(INT5_vect) // 스위치 인터럽트 발생시 모드를 변경한다.(sleep 모드 전환)
{
	offFnd();
	if (slp_mode == 0) // sleep모드가 아닌 상태에서 스위치를 누르면 sleep모드가 켜지고 wake모드는 꺼진다.
	{
		slp_mode = 1;
		wake_mode = 0;
	}
	else if (slp_mode == 1) {
		slp_mode = 0; // off상태로 전환
	}
	_delay_ms(10);
}
void offFnd() { // 모든 LED등을 소등한다.
	{
		PORTC = 0;
		PORTG = 0x01;
		PORTG = 0x02;
		PORTG = 0x04;
		PORTG = 0x08;
		PORTA = 0x00;
	}
}
int main(void)
{
	INT8U   err;
	OSInit();
	OS_ENTER_CRITICAL();

	TCCR0 = 0x05;
	TIMSK = _BV(TOIE0);
	TCNT0 = MI;
	DDRB = 0x10;

	DDRE = 0xCF; // E포트를 스위치 입력포트로 사용
	EICRB = 0x0A; // 인터럽트 4,5의 trigger을 하강에지로 설정
	EIMSK = 0x30; // 인터럽트 4,5 의 enable
	SREG |= 1 << 7; // 전체 인터럽트 

	OS_EXIT_CRITICAL();
	
	// 동기화 작업을 위한 mail box, mutex, semaphore, message queue 등을 모드 create한다.
	MboxTemper = OSMboxCreate(0);
	Mutex = OSMutexCreate(0, &err);
	MboxLight = OSMboxCreate(0);
	mySem = OSSemCreate(1);
	Lqueue = OSQCreate(&LqTbl[0], 20);
	test = OSMutexCreate(0, &err);

	// 각 Task들을 실행시킨다.
	OSTaskCreate(TemperatureTask, (void*)0, (void*)& TaskStk[0][TASK_STK_SIZE - 1], 1);
	OSTaskCreate(LightTask, (void*)0, (void*)& TaskStk[1][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(temperTask, (void*)0, (void*)& TaskStk[2][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(adcTask, (void*)0, (void*)& TaskStk[4][TASK_STK_SIZE - 1], 4);
	OSTaskCreate(FndDisplayTask, (void*)0, (void*)& TaskStk[3][TASK_STK_SIZE - 1], 5);

	OSStart();

	return 0;
}

void InitI2C()
{
	PORTD = 3;						// For Pull-up override value
	SFIOR &= ~(1 << PUD);			// PUD
	TWSR = 0;						// TWPS0 = 0, TWPS1 = 0
	TWBR = 32;                      // for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);   // TWEA = Ack pulse is generated
}

int ReadTemperature(void)
{
	int value;

	OS_ENTER_CRITICAL();

	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
	while (!(TWCR & _BV(TWINT)));

	TWDR = 0x98 + 1; //TEMP_I2C_ADDR + 1
	TWCR = _BV(TWINT) | _BV(TWEN);
	while (!(TWCR & _BV(TWINT)));

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while (!(TWCR & _BV(TWINT)));

	// 온도센서는 16bit 기준으로 값을 가져오므로
	// 8비트씩 2번을 받아야 한다.
	value = TWDR << 8;
	TWCR = _BV(TWINT) | _BV(TWEN);
	while (!(TWCR & _BV(TWINT)));

	value |= TWDR;
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);

	value >>= 8;

	TIMSK = (value >= 33) ? TIMSK | _BV(TOIE2) : TIMSK & ~_BV(TOIE2);

	OS_EXIT_CRITICAL();

	return value;
}

void TemperatureTask(void* data)
{
	int   value;
	INT8U   err;
	data = data;
	InitI2C();
	DDRA = 0xFF;

	while (1) {
		if (!slp_mode) {
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		value = ReadTemperature();
		FndNum = value;
		OSMboxPost(MboxTemper, &value); // 메일박스를 사용하여 온도값을 넘겨준다. 이를 아래의 temperTask에서 pend한다.

		DDRB = 0x10;
		int i;

		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

void temperTask(void* data)
{
	INT8U   err;
	data = data;
	// TemperatureTask의 값을 받은 후,
	// OSTimeDlyHMSM()을 통해 1초 딜레이를 발생하고
	// LightTask의 값을 받도록 수행
	while (1) {
		if (!slp_mode) {
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		int val = *(int*)OSMboxPend(MboxTemper, 0, &err); // 온도값을 받아온다.
		FndNum = val;
		// 온도에 따라 cur_state를 변화시켜준다. 이때 cur_state 공유자원에 접근하기 위해 세마포어를 사용한다.
		OSSemPend(mySem, 0, &err);
		if (val > 29) {
			cur_state = 5;
		}
		else if (val <= 28) {
			cur_state = 4;
		}
		else {
			cur_state = 1;
		}
		OSSemPost(mySem);
		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

void FndDisplayTask(void* data) // fnd display를 위한 Task
{
	unsigned char FND_DATA[] = { // 화면에 띄우기위한 숫자와 알파벳을 표현하였다.
	   0x3f, // 0
	   0x06, // 1 
	   0x5b, // 2 
	   0x4f, // 3 
	   0x66, // 4 
	   0x6d, // 5 
	   0x7d, // 6 
	   0x27, // 7 
	   0x7f, // 8 
	   0x6f, // 9 
	   0x77, // A 
	   0x7c, // B 
	   0x39, // C 
	   0x5e, // D 
	   0x79, // E 
	   0x71, // F 
	   0x76, // H
	   0x07, // T
	   0x38, // L
	   0x6d, // S
	   0x73, // P
	   0x37, // N
	   0x80, // . 
	   0x40, // - 
	   0x08  // _
	};
	unsigned int num0, num1, num2, num3;
	INT8U   err;

	data = data;

	DDRC = 0xff;
	DDRG = 0x0f;

	
	/* loop 시작 */
	/*
	 이곳에서 모드를 판단하여 sleep모드의 경우 온도센서를 통하여 온도 값을
	 받아오고 cooling system과 heating system중 어떤 것을 가동할지 결정
	 하고 fnd display를 통해 user에게 보여준다.
	*/
	while (1) {
		int count = 1;
		// cur_state 공유자원을 사용하기 위한 세마포어 사용
		OSSemPend(mySem, 0, err);
		count = cur_state;
		OSSemPost(mySem);
		// off 상태 (아무 것도 작동하지 않고 fnd에 off를 띄워준다.)
		if (slp_mode == 0 && wake_mode == 0) {
			PORTC = FND_DATA[0];
			PORTG = 0x04;
			_delay_ms(1);
			PORTC = FND_DATA[15];
			PORTG = 0x02;
			_delay_ms(1);
			PORTC = FND_DATA[15];
			PORTG = 0x01;
			_delay_ms(1);
			continue;
		}
		// sleep 모드
		else if (slp_mode == 1) {
			if (count == 1) {
				//SLEP : 수면모드로 잠자기에 적절한 온도를 유지하는 상태
				PORTC = FND_DATA[19];
				PORTG = 0x08;
				_delay_ms(1);
				PORTC = FND_DATA[18];
				PORTG = 0x04;
				_delay_ms(1);
				PORTC = FND_DATA[14];
				PORTG = 0x02;
				_delay_ms(1);
				PORTC = FND_DATA[20];
				PORTG = 0x01;
				_delay_ms(1);

			}
			else if (count == 4) {
				// HEAT : 난방 시스템 가동
				PORTC = FND_DATA[16];
				PORTG = 0x08;
				_delay_ms(1);
				PORTC = FND_DATA[14];
				PORTG = 0x04;
				_delay_ms(1);
				PORTC = FND_DATA[10];
				PORTG = 0x02;
				_delay_ms(1);
				PORTC = FND_DATA[17];
				PORTG = 0x01;
				_delay_ms(1);
			}
			else if (count == 5) {
				// COOL : 냉방 시스템 가동
				PORTC = FND_DATA[12];
				PORTG = 0x08;
				_delay_ms(1);
				PORTC = FND_DATA[0];
				PORTG = 0x04;
				_delay_ms(1);
				PORTC = FND_DATA[0];
				PORTG = 0x02;
				_delay_ms(1);
				PORTC = FND_DATA[18];
				PORTG = 0x01;
				_delay_ms(1);
			}
		}
		// wake 모드
		else if (wake_mode == 1) { 
			if (count == 2){
				// open
				PORTC = FND_DATA[0];
				PORTG = 0x08;
				_delay_ms(1);
				PORTC = FND_DATA[20];
				PORTG = 0x04;
				_delay_ms(1);
				PORTC = FND_DATA[14];
				PORTG = 0x02;
				_delay_ms(1);
				PORTC = FND_DATA[21];
				PORTG = 0x01;
				_delay_ms(1);
			}
			else if (count == 3) {
				// close
				PORTC = FND_DATA[12];
				PORTG = 0x08;
				_delay_ms(1);
				PORTC = FND_DATA[18];
				PORTG = 0x04;
				_delay_ms(1);
				PORTC = FND_DATA[0];
				PORTG = 0x02;
				_delay_ms(1);
				PORTC = FND_DATA[19];
				PORTG = 0x01;
				_delay_ms(1);
			}
		}
	}
}

// 광센서 초기화
void init_adc()
{
	DDRF = 0x01;
	ADMUX = 0x00;
	ADCSRA = 0x87;
}

// 광센서에서 값을 받아오는 함수
unsigned short read_adc()
{
	unsigned char adc_low, adc_high;
	unsigned short value;
	ADCSRA |= 0x40;
	while ((ADCSRA & 0x10) != 0x10);
	adc_low = ADCL;
	adc_high = ADCH;
	value = ((unsigned short)adc_high << 8) | (unsigned short)adc_low;
	return value;
}

// 광센서 값을 읽고 전달해주는 Task
void LightTask(void* data)
{
	data = data;
	unsigned short value;
	init_adc();

	while (1)
	{
		if (!wake_mode) { // wake mode가 아닐경우 작동하지 않는다.
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		value = read_adc(); // value에 현재 조도값을 받아온다.
		OSQPost(Lqueue, value); // 메세지큐를 사용하여 post한다. 이를 아래의 adc task가 pend하여 사용한다.
		OSTimeDlyHMSM(0, 0, 0, 900);
	}
}

// 광센서 값을 받아와 상황에 따라 처리해주는 Task
void adcTask(void* data) {
	data = data;
	INT8U err;
	
	while (1) {
		if (!wake_mode) {
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		unsigned short msg;
		msg = (unsigned short)OSQPend(Lqueue, 0, &err); // 메세지 큐를 통해 광센서에서 측정한 값을 가져온다.
		
		OSSemPend(mySem, 0, &err); // cur_state값에 접근하기 위한 세마포어 사용
		if (msg < 4) // 어두울 때 
		{
			// 이전상태와 비교하여 현재상태가 이전과 달라질 때 부저를 울린다.
			// 이를위해 이전상태를 저장하는 pre_state변수를 만들어 cur_state값과 비교한다.
			if (pre_state == 3) { 
				// state의 변화 발생
				int i;
				// 반복문을 사용하여 부저를 울려준다.
				for (i = 0; i < 4; i++) {
					PORTB = 0x10;
					_delay_ms(1);
					PORTB = 0x00;
					_delay_ms(1);

				}
			}
			pre_state = cur_state;
			cur_state = 2;
			if (msg >= 3)
			{
				PORTA = 0x3c;
			}
			else if (msg >= 2)
			{
				PORTA = 0x7e;
			}
			else if (msg >= 0)
			{
				PORTB = 0x10;
				PORTA = 0xff;
			}
		}
		else // 밝을 때
		{
			if (pre_state == 2) {
				// state 변화 발생
				int i;
				// 반복문을 사용하여 부저를 울린다.
				for (i = 0; i < 4; i++) {
					PORTB = 0x10;
					_delay_ms(1);
					PORTB = 0x00;
					_delay_ms(1);

				}
			}
			pre_state = cur_state;
			cur_state = 3;
			PORTA = 0x18;
		}
		OSSemPost(mySem); // state사용을 마치고 세마포어를 포스트한다.
		OSTimeDlyHMSM(0, 0, 0, 900);
	}
}