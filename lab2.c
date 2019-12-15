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
	�ý��ۿ��� ���� �߿��� ������ cur_state�̴�. �����鿡�� ���� �޾ƿͼ� state���� ��ȭ��Ű��
	�׿� ���� fnd display�� led, �������� �۵���Ų��. 
	state�� 0, 1, 2, 3, 4, 5�� ���� �����µ� ������� off, sleep, open, close, heat, cool�� �ǹ̸� ��Ÿ����.
	������ �ǹ̴� ������ ����.

	off : �ý����� �����ִ� ����
	sleep : �ù�� ����ý��� ������ �ʿ����� ���� ���·� ���鿡 ������ �µ��� ������ ����(sleep mode)
	open : ������ ���� Ŀư�� open�� ����(wake mode)
	close : ������ ���� Ŀư�� close�� ����(wake mode)
	heat : ����ý����� ������ ����(sleep mode)
	cool : �ù�ý����� ������ ����(sleep mode)
*/
volatile INT8U FndNum, CDS, cur_state, slp_mode = 0, wake_mode = 0, pre_state = 0;
volatile INT8U mus = 0, mel_idx = 0;

void  TemperatureTask(void* data);
void  temperTask(void* data);
void  FndDisplayTask(void* data);
void  LightTask(void* data);
void  adcTask(void* data);
void* LqTbl[20];

ISR(INT4_vect) // ����ġ ���ͷ�Ʈ �߻��� ��带 �����Ѵ�.(wake ��� ��ȯ)
{
	offFnd();
	if (wake_mode == 0) // wake��尡 �ƴ� ���¿��� ����ġ�� ������ wake���� ��ȯ�Ѵ�.
	{
		wake_mode = 1;
		slp_mode = 0;
	}
	else if (wake_mode == 1) {

		wake_mode = 0;
	}
	_delay_ms(10);
}

ISR(INT5_vect) // ����ġ ���ͷ�Ʈ �߻��� ��带 �����Ѵ�.(sleep ��� ��ȯ)
{
	offFnd();
	if (slp_mode == 0) // sleep��尡 �ƴ� ���¿��� ����ġ�� ������ sleep��尡 ������ wake���� ������.
	{
		slp_mode = 1;
		wake_mode = 0;
	}
	else if (slp_mode == 1) {
		slp_mode = 0; // off���·� ��ȯ
	}
	_delay_ms(10);
}
void offFnd() { // ��� LED���� �ҵ��Ѵ�.
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

	DDRE = 0xCF; // E��Ʈ�� ����ġ �Է���Ʈ�� ���
	EICRB = 0x0A; // ���ͷ�Ʈ 4,5�� trigger�� �ϰ������� ����
	EIMSK = 0x30; // ���ͷ�Ʈ 4,5 �� enable
	SREG |= 1 << 7; // ��ü ���ͷ�Ʈ 

	OS_EXIT_CRITICAL();
	
	// ����ȭ �۾��� ���� mail box, mutex, semaphore, message queue ���� ��� create�Ѵ�.
	MboxTemper = OSMboxCreate(0);
	Mutex = OSMutexCreate(0, &err);
	MboxLight = OSMboxCreate(0);
	mySem = OSSemCreate(1);
	Lqueue = OSQCreate(&LqTbl[0], 20);
	test = OSMutexCreate(0, &err);

	// �� Task���� �����Ų��.
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

	// �µ������� 16bit �������� ���� �������Ƿ�
	// 8��Ʈ�� 2���� �޾ƾ� �Ѵ�.
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
		OSMboxPost(MboxTemper, &value); // ���Ϲڽ��� ����Ͽ� �µ����� �Ѱ��ش�. �̸� �Ʒ��� temperTask���� pend�Ѵ�.

		DDRB = 0x10;
		int i;

		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

void temperTask(void* data)
{
	INT8U   err;
	data = data;
	// TemperatureTask�� ���� ���� ��,
	// OSTimeDlyHMSM()�� ���� 1�� �����̸� �߻��ϰ�
	// LightTask�� ���� �޵��� ����
	while (1) {
		if (!slp_mode) {
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		int val = *(int*)OSMboxPend(MboxTemper, 0, &err); // �µ����� �޾ƿ´�.
		FndNum = val;
		// �µ��� ���� cur_state�� ��ȭ�����ش�. �̶� cur_state �����ڿ��� �����ϱ� ���� ������� ����Ѵ�.
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

void FndDisplayTask(void* data) // fnd display�� ���� Task
{
	unsigned char FND_DATA[] = { // ȭ�鿡 �������� ���ڿ� ���ĺ��� ǥ���Ͽ���.
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

	
	/* loop ���� */
	/*
	 �̰����� ��带 �Ǵ��Ͽ� sleep����� ��� �µ������� ���Ͽ� �µ� ����
	 �޾ƿ��� cooling system�� heating system�� � ���� �������� ����
	 �ϰ� fnd display�� ���� user���� �����ش�.
	*/
	while (1) {
		int count = 1;
		// cur_state �����ڿ��� ����ϱ� ���� �������� ���
		OSSemPend(mySem, 0, err);
		count = cur_state;
		OSSemPost(mySem);
		// off ���� (�ƹ� �͵� �۵����� �ʰ� fnd�� off�� ����ش�.)
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
		// sleep ���
		else if (slp_mode == 1) {
			if (count == 1) {
				//SLEP : ������� ���ڱ⿡ ������ �µ��� �����ϴ� ����
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
				// HEAT : ���� �ý��� ����
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
				// COOL : �ù� �ý��� ����
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
		// wake ���
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

// ������ �ʱ�ȭ
void init_adc()
{
	DDRF = 0x01;
	ADMUX = 0x00;
	ADCSRA = 0x87;
}

// ���������� ���� �޾ƿ��� �Լ�
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

// ������ ���� �а� �������ִ� Task
void LightTask(void* data)
{
	data = data;
	unsigned short value;
	init_adc();

	while (1)
	{
		if (!wake_mode) { // wake mode�� �ƴҰ�� �۵����� �ʴ´�.
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		value = read_adc(); // value�� ���� �������� �޾ƿ´�.
		OSQPost(Lqueue, value); // �޼���ť�� ����Ͽ� post�Ѵ�. �̸� �Ʒ��� adc task�� pend�Ͽ� ����Ѵ�.
		OSTimeDlyHMSM(0, 0, 0, 900);
	}
}

// ������ ���� �޾ƿ� ��Ȳ�� ���� ó�����ִ� Task
void adcTask(void* data) {
	data = data;
	INT8U err;
	
	while (1) {
		if (!wake_mode) {
			OSTimeDlyHMSM(0, 0, 1, 0);
			continue;
		}
		unsigned short msg;
		msg = (unsigned short)OSQPend(Lqueue, 0, &err); // �޼��� ť�� ���� ���������� ������ ���� �����´�.
		
		OSSemPend(mySem, 0, &err); // cur_state���� �����ϱ� ���� �������� ���
		if (msg < 4) // ��ο� �� 
		{
			// �������¿� ���Ͽ� ������°� ������ �޶��� �� ������ �︰��.
			// �̸����� �������¸� �����ϴ� pre_state������ ����� cur_state���� ���Ѵ�.
			if (pre_state == 3) { 
				// state�� ��ȭ �߻�
				int i;
				// �ݺ����� ����Ͽ� ������ ����ش�.
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
		else // ���� ��
		{
			if (pre_state == 2) {
				// state ��ȭ �߻�
				int i;
				// �ݺ����� ����Ͽ� ������ �︰��.
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
		OSSemPost(mySem); // state����� ��ġ�� ������� ����Ʈ�Ѵ�.
		OSTimeDlyHMSM(0, 0, 0, 900);
	}
}