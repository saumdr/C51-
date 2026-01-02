/**下面的代码主要用于51的基本配置，每一部分分别由主程序和调用方式来组成，包括了“数码管、延时函数、数显函数、按键配置（消抖、非阻塞、长按短按、矩阵键盘一键配置）、定时器配置及说明、外部中断、波特率串口配置及说明、XBYTE的用法及外部地址的设定、外部扩展芯片82C55、ADC0809测定电压电路及代码、DAC0832产生各种波形的连线（包括正弦波、方波、三角波等）”及其余一些便捷操作，欢迎大家提出问题和建议**/

#define uint  unsigned int
#define uchar unsigned char


数码管段码
````c
uchar seg[] = {};        // 存储在RAM中
code uchar seg[] = {};   // 和下面一样，存储在flash中，推荐使用
uchar code seg[] = {};

````

延时函数
````c
void delay_ms(uint ms)
{
    uint i, j;  
    for(i=0; i<ms; i++)     //这里顺序不要反了
        for(j=0; j<120; j++); //每个for循环迭代约为8个机器周期，也就是 8*120*1us 约等于 1ms
}

void delay_10us()
{
    _nop_; _nop_; _nop_;    //需要加上头文件<intrins.h>
    _nop_; _nop_; _nop_;
}
或者
void delay_10us(uint us)
{
    uint i;
    while(us--)
    {
        i = 2;
        while(i--);
    }
}
````

数码管显示配置，共阳极选中是1，共阴极选中是0
````c
code uchar seg[] = {}；     //段码0~9
uchar Display[] = {0, 0, 0, 0}；    //显示的对应四位
//调用
Display[i] = your_data;

/*最常用的版本：P1^0~P1^3位选，P2数选，但会影响到P1高四位*/
void display()
{
    uint i;
    for(i=0; i<4; i++)  //这里假设只用了前四位
    {
        P2 = 0xff;  //共阳极的消隐，共阴极是0x00

        P2 = seg[Display[i]];
        P1 = 1 << i;    //如果是共阴极，需要~(1 << i)

        delay_ms(1);
    }
}

/*改进1：使用掩码来防止影响高四位*/
void display()
{
    uint i, pos = 0x01;
    P1 = P1 & 0xf0;
    for(i=0; i<4; i++)
    {
        P2 = 0xff;

        P2 = seg[Display[i]];
        P1 = (P1 & 0xf0) | pos;
        pos = pos << 1;
        
        delay_ms(1);
    }
}

/*改进2：定时器扫描*/
void int0() interrupt 1
{
	static uint i;
	
	TH0 = (65536 - 1000) / 256;
	TL0 = (65536 - 1000) % 256; //1ms
	
	P2 = 0xff;
	P2 = seg[Display_num[i]];
	P1 = 1 << i;
	i++;
	if(i >= 4)	i = 0;  
}

//调用
void main()
{
    while(1)
    {
        for(i=0; i<20; i++)
        {
            display();
        }
    }
}

//注意，如想要显示小数点，对于共阳极需要 P2 = P2 & 0x7f，共阴极需要 P2 = P2 | 0x80
````

按键配置    //一般可以放在定时器中扫描
````c
sbit bt = P1^0;
uint key_flag =0;

/*最常用的配置：消抖+识别单个按键*/
void key_scan()
{
    if(bt == 0)
    {
        delay_ms(1);    //延时消抖
        if(bt == 0) key_flag = 1;
    }
}
或者
void key_scan()
{
    if(bt == 0)
    {
        while(!bt)  key_flag = 1;
    }
}

//调用
void main()
{
    while(1)
    {
        key_scan();
        switch(key_flag)    //假设有多个按键
        {
            case 1: //action1
            case 2: //action2
            ...
        }
    }
}
/*改进1：消抖+长按*/
void key_scan()
{
    static uint key_count;  //使用静态变量来存储按键按下时间，也可在外部直接定义
    if(bt == 0) key_count++;
    if(bt == 1 && key_count > 0)    //松开的时候判断
    {
        if(key_count >= 20) key_flag = 2;   //定义为长按
        else if(key_flag == 0)  key_flag = 1;   //定义为短按
        else key_flag = 0;  //短按
        
        key_count = 0;  //一定要清除
    }
}

//调用
void main()
{
    while(1)
    {
        for(i=0; i<20; i++)
        {
            delay(50);  //此处的值与长按的逻辑配合，50*20=1s为长按阈值，如果这里不写for，最好把长按阈值改为500或者更多一些
            key_scan();
        }
        if(key_flag)    
        {
            //状态机
        }
    }
}

/*矩阵键盘，连接了P1*/
uint code Key_Map_Table[] = {
    1, 2, 3, 15,  // 第1行
    4, 5, 6, 14,  // 第2行
    7, 8, 9, 13,  // 第3行
    10, 0, 11, 12 // 第4行
};

uint key_scan() 
{
    uint i, j;
    
    for(i = 0; i < 4; i++) // 循环4行
    {
        P1 = ~(1 << i); // 拉低第i行 (0xFE, 0xFD, 0xFB, 0xF7)
        
        for(j = 0; j < 4; j++) // 循环4列
        {
            if((P1 & (0x10 << j)) == 0) // 检测第j列 (0x10, 0x20, 0x40, 0x80)
            {
                delay_ms(10); // 消抖
                if((P1 & (0x10 << j)) == 0)
                {
                    // 防止一次按下被识别多次
                    while((P1 & (0x10 << j)) == 0);
                    
                    // 计算索引并查表返回真实键值
                    return Key_Map_Table[i * 4 + j];
                }
            }
        }
    }
    return 0xFF; // 无按键
}

//调用
key = key_scan();
````

定时器配置

````c
/*对于定时器的ISR映射表如下：
    interrupt 0 -> 外部中断0（INT0|P3.2）
    interrupt 1 -> 定时器0(无论是计数器还是定时器)
    interrupt 2 -> 外部中断1（INT1|P3.3）
    interrupt 3 -> 定时器1
    interrupt 4 -> 串行口1中断
    ...
*/
/*定时器0初始化*/
//定时器0
void Timer0_Init()
{
    TMOD &= 0xF0;   //先清空定时器0配置
    TMOD |= 0x01;   //TMOD = 0000 0001 ，高四位是定时器1，低四位是定时器0。以定时器0为例，第一位：GATE=0，则定时器启动只由TR0决定，GATE=1，由TR0和INT0（外部触发）决定；第二位清零是定时器，1位计数器（由P3.4触发）；后两位设置定时器模式，常用01（模式1：16位）
    TH0 = (65536-1000) / 256;
    TL0 = (65536-1000) % 256;   //1ms

    EA = 1;     //总开关
    ET0 = 1;    //中断允许
    TR0 = 1;    //启动定时器
}

//调用
void timer0() interrupt 1
{
    TH0 = (65536-1000) / 256；
    TL0 = (65536-1000) % 256;   //1ms，旧的51在16位模式下不会重装载
    ...
}

//计数器，触发引脚是P3.4(IT0)，每来一个下降沿触发一次
void Counter0_Init()
{
    TMOD &= 0xF0;   //先清空定时器0配置
    TMOD |= 0x05；

    TH0 = (65536 - 100 ) / 256；
    TL0 = (65536 - 100 ) % 256;     //外部脉冲100次后溢满，有标志位 TF0 = 1，

    TR0 = 1;
    EA = 1;

    ET0 = 1;    //使能定时器0中断，如果用中断模式就开启，如果不用中断可以用TF0来检测
}

//调用
void counter0() interrupt 1 //中断调用
{
    TH0 = (65536 - 100 ) / 256；
    TL0 = (65536 - 100 ) % 256;

    //处理计数完成事件
}
或
void main()
{
    while(1)
    {
        if(TF0)
        {
            TF0 = 0;
            //处理溢出事件
        }
    }
}
````


外部中断
````c
//配置
void EXterrupt0_Init()
{
    IT0 = 1;    //1为下降沿触发，0为低电平触发
    EX0 = 1；   //使能外部中断
    EA = 1;   
}

//中断服务函数
void EX0_ISR() interrupt 0
{
    //如果是低电平触发需要软件清除 IE0，如果是下降沿则不用
 //   IE0 = 0;    //这里写处理函数，当外部引脚触发时会进来
}

void main()
{   
    while(1)
    {
        if(INT0 == 1)   //如果外部中断对应的引脚松开
        {
            EX0 = 1;    //需要重新给1！！！
        }
    }
}
````


串口和波特率（硬件：RXD和TXD）
````c
/*主要寄存器/位解释：
1.SCON：串行口控制寄存器，主要包括RI（B0，串口1接收中断标志，必须手动清零）、TI（B1，串口1发送中断标志）、REN（=1允许接收，手动复位）SM1和SM0（B7和B6，控制工作模式：模式0（移位寄存器，波特率固定SYScLK/12），模式1（常用，8位可变波特率串口，计算方式：（2^SMOD/32）/（定时器1的溢出率）），模式2（九位波特率固定），模式3（九位波特率可变，计算方式：（2^SMOD/32）/（定时器1的溢出率））。一般设置位SCON = 0x50（模式1，允许接收，清除两个中断标志位）
2.SBUF:串口数据缓存寄存器，实际上有两个SBUF，一个接收一个发送。当数据的最后一位从“发送数据缓存寄存器”移出去的时候，置标志位 TI = 1；如果数据的最后一位移进“接收数据缓存寄存器”时候，置标志位 IR = 1
3.SMOD：PCON的最高位，用于设置波特率是否加倍（0：不加倍；1：*2），默认就好（0）
4.定时器1的溢出率计算：定义为单位时间（1s）内定时器1溢出的次数
    1.机器周期时间 = 12 / fosc（晶振） (秒)
    2.每次计数时间 = 机器周期时间
    3.计数次数 = 256 - TH1（因为用的是八位重装载模式，只用高八位）
    4.溢出时间 = 计数次数 × 机器周期时间
            = (256 - TH1) × (12 / fosc) (秒)
    5.溢出率 = 1 / 溢出时间
            = fosc / [12 × (256 - TH1)] (Hz)
    6.常用（记住这个）  TH1 = 253（0xFD)，得到9600bps
5.interrupt 0 外部中断服务函数是一个双向函数，可以是接收或者触发时（即 TI = 1 或者 IR = 1 的时候触发），所以触发后需要区别这两个事件
*/
//初始化
void UART_Init()
{
    TMOD &= 0x0F;   //清除定时器1的配置
    TMOD |= 0x20;   //模式1的定时器，八位重装载（不要重新在中断服务函数中配置）
    TH1 = TL1 = 0xFD;   //(9600bsp)
    TR1 = 1;    //启动定时器1

    SCON = 0x50；   //配置位串口模式1、允许接收

    ES = 1;     //允许串口中断
    EA = 1；    //总开关
}

//调用
uchar send_data, recv_data, dat;    //dat是要显示的变量

void main()
{
    while(1)
    {
        SBUF = send_data;
        if(uart_flag)
        {
            uart_flag = 0;
            dat = SBUF; 
        }
    }
}

void UART_ISR(void) interrupt 4
{
    if(RI)  // 接收中断
    {
        RI = 0;                  // 清除标志
        recv_data = SBUF;        // 保存数据
        uart_flag = 1;           // 设置标志
    }
    if(TI)  TI = 0;             //需要单独判断
}

或者

void UART_ISR() interrupt 4 
{
    if(RI)  //接收中断
    {
        RI = 0;
        recv_data = SBUF;   //接收数据
        
        SBUF = send_data;   //发送数据
        while(!TI);     //等待发送完成
        TI = 0； 
    }
}

/**
 * 发送一个字节（查询方式）
 */
void UART_SendByte(uchar send_data)
{
    SBUF = send_data;     // 写入发送缓冲区
    while(!TI);     // 等待发送完成
    TI = 0;         // 清除发送标志
}
````

XBYTE的用法
````c
//使用格式
#include "absacc.h"
#define ADDR XBYTE[]    //16位地址
/*说明：指向外部RAM（xdata）起始地址的一个指针，运行后CPU会执行下面操作：
    执行写操作时：
    1.CPU自动把地址低8位放到P0，高四位放到P2
    2.硬件自动发出ALE锁存地址
    3.CPU自动把数据放到P0
    4.硬件自动发出WR脉冲

    执行读操作时：
    1.CPU自动把地址低8位放到P0，高四位放到P2
    2.硬件自动发出ALE锁存地址
    3.硬件自动发出RD脉冲
    4.外部设备把数据放到P0
    5.CPU自动读取P0上的数据
*/

//使用操作
ADDR = //你写入的数据，即指定好地址后就可以当成普通的IO口直接使用
//注意：此时P0是推挽输出模式，不用上拉电阻
````


外部扩展芯片———8255A
````c
//主要引脚及寄存器作用
1.D0~D7：接收来自P0数据
2.A0、A1：地址线

3.控制寄存器（8位）：D7通常用1；
                    D6、D5为A组方式（00最常用：IO输入/输出；01：选通；1X：双向选通）；
                    D4：A口IO； //指的是当D4 = 0时表示A为输出，1则为输入，下面的一样
                    D3：C低四位IO;
                    D2：B组方式；
                    D1：B口IO；
                    D0：C高四位IO；
    即如果想要PA、PB、PC都是输出，则该寄存器设为 COM8255 = 0x80；

//主要接线方式及地址说明
1.P0 -> D0~D7（包括74LS373的，用于所存地址；8255A的：用于读取/写入数据）
2.ALE -> 74LS373的LE
3.WR/RD -> 8255A的WR和RD

4.74LS373的Q0、Q1 -> 8255A的A0、A1   //用与地址选择线，指定了CPU访问哪一个端口
                                    //  A0 A1 | 端口
                                    //  0  0  | PA
                                    //  0  1  | PB
                                    //  1  0  | PC
                                    //  1  1  | 控制寄存器
5.P2的任意三位（通常是高三位） -> 74LS138 --> 74LS138的一个输出 -> 8255A的CS  //当CS为低电平时，即选中，若Y7 -> CS，即74LS138的CBA = 111

综上所述，如果使用P2.7~P2.5控制CS，Q0、Q1控制A0、A1，则有（除了高三位（P2.7~P2.5）和低两位（P0.0~P0.1），其余位任意）
#define PA_ADDR   XBYTE[0xE000] //可以换成0xE550
#define PB_ADDR   XBYTE[0xE001]
#define PC_ADDR   XBYTE[0xE002]
#define COM8255   XBYTE[0xE003] //控制寄存器

//使用模板，使用P2.7~P2.5控制CS，Q0、Q1控制A0、A1
#define PA_ADDR   XBYTE[0xE000] //
#define PB_ADDR   XBYTE[0xE001]
#define PC_ADDR   XBYTE[0xE002]
#define COM8255   XBYTE[0xE003] //控制寄存器

void init_8255(void)
{
	COM8255 = 0x80;	//配置ABC工作模式
}
````

ADC测量电压——ADC0809
````c
//地址说明
1.ADD A~C指向测定电压的输入口   //如果是用IN3，则是对应 CBA = 011
2.高位常通过P2.7~P2.5连接74LS138进行片选，CS为低电平选通（但电路一般不太一样）

//引脚说明
1.START：用于启动ADC转换，输入一个正脉冲启动
2.OE：数据输出允许，当AD转换结束时候，这个引脚输入一个高电平才能让0809输出转换结果
3.EOC：AD转换结束信号，转换结束后，输出一个高电平，转换期间一直是低电平
4.CLK：需要时钟来进行控制AD转换，要求不高于640KHZ（我仿真用500KHZ）
5.VREF+和VREF-：参考电压输入    //作用：设定测量电压参考值
                                //数字输出值 = (输入电压 / 参考电压) × 255

                                // 例如：
                                // Vref = 5.00V, Vin = 2.50V
                                // 数字值 = (2.50 / 5.00) × 255 = 127.5 ≈ 127

                                // Vref = 2.50V, Vin = 2.50V  
                                // 数字值 = (2.50 / 2.50) × 255 = 255
6.ALE：地址所存

//引脚连接
1.P0 -> D0~D7（包括74LS373的，用于锁存地址；0809的八位输出：注意这里需要反接，即P0.0接到OUT8）
2.CPU的ALE -> 74LS373的LE
3.CPU的WR和74LS138的选通位 -或非门> 0809的ALE和START    //如果你不用74LS138，那么可以直接把WR通过非门连接到ALE和START
4.CPU的RD和74LS138的选通位 -或非门> 0809的OE            //如果你不用74LS138，那么可以直接把RD通过非门连接到OE，程序一个样，因为START、ALE和OE都是高电平触发，而写入地址的时候CPU的WR和RD是给低电平

5.0809的CLOCK接入时钟脉冲
6.VREF+和VCC接到正电压，VREF-和GND接到GND
7.74LS373的Q0~Q2接到0809的ADD A~C

//使用模板
#define ADC XBYTE[0xE003]  //这里是IN3测电压，Y7为74LS138输出

void main()
{
    while(1)
    {
        uint temp, result;

         //1.启动转换 (绝对不能注释掉！)
        // 对端口写入任意数据，产生 WR 负脉冲，触发 ADC 的 START 和 ALE
        ADC = 0;

        //2.等待转换 (ADC0808 转换需要时间)
        delay_ms(1);   //或者用 while(!EOC);

        //3.此时resullt为0，产生 RD 负脉冲，触发 ADC 的 OE 位，打开三态门，把转换好的 8位数据 放到 D0-D7 总线上
        temp = ADC;

        //数据转换
        result = (uint)temp * 50 / 255; //（0809是8位AD转换，这里把0~255转为0~50）

        for(i = 0; i <50; i++) 
        {
            display(result);    //这里你可以显示一下
        }
    }
}
````

DAC产生波形——DAC0832
````c
//地址说明
只需要关注怎么给CS低电平就好，比如接到74LS138的Y7，则 CBA = 111

//引脚说明
1.DI0~DI7：8位数据输入线，TLL电平；
2.ILE：数据锁存允许控制信号输入线，高电平有效；
3.CS：片选信号输入线(选通数据锁存器)，低电平有效；
4.WR1：为输入寄存器的写选通信号；
5.XFER：数据传送控制信号输入线，低电平有效；
6.WR2 ：为DAC寄存器写选通输入线；
7.Iout1：电流输出线。当输入全为1时Iout1最大；
8.Iout2：电流输出线。其值与Iout1之和为一常数；
9.Rfb：反馈信号输入线,芯片内部有反馈电阻；
10.Vcc：电源输入线（范围为+5v~+15v）；
11.Vref：基准电压输入线（-10v~+10v），为反向！！！；
12.AGND：模拟地,摸拟信号和基准电源的参考地；
13.AGND：数字地,两种地线在基准电源处共地比较好；

//一般电路接线，单缓存模式
1.WR1 -> CPU的WR
2.ILE -> VCC
3.WR2、XFER -> GND
4.IOUT2 -> GND和运算的+输入
5.IOUT1 -> 运算的-输入
6.RFB -> 运算输出
7.DI0~7 -> P0（可不带上拉，此时为强推挽输出模式）
8.VREF -> 激励电压源（基准电压），作用：、决定输出电压范围和精度，计算公式为： Vout = - Vref * DATA / 256。比如VREF接-5V，输出范围就是0~+5V；接+5V，输出就是 0~-5V（反相输出）。
9.运放接线（LM358N）：用来电流->电压转换
10.CS -> 地址选通
//第一道门：CS=0 ILE=1 WR=0(输入锁存器)；第二道门：WR2=0 XFER=0(DAC寄存器)

//其他工作方式接线（只写与上面不同的）
/*双缓冲：多个DAC*/
1.XFER -> CPU的IO口控制
2.WR2 -> WR或接地   //作用：当所有数据都暂存好后，CPU控制XFER信号变低，所有DAC的“第二道门”同时打开，输出同时变化。

//使用模板
#define DAC XBYTE[0xE000]   //接到了Y7

//最重要就是这里
void Write_DAC(uchar value)
{
    DAC = value;      // 直接写P0口
}

//产生矩形波
void generate_Square()
{
	Write_DAC(20);
	delay(10);
	Write_DAC(0);
	delay(10);
}

//产生三角波
void generate_Triangular()
{
	u16 i;
	for(i=0; i<20; i++)
	{
		Write_DAC(i);
		delay_10us(50);		//500us *20 = 10ms
	}
	
	for(i=20; i>0; i--)
	{
		Write_DAC(i);
		delay_10us(50);
	}
}

//产生梯形波
void generate_Trapezoid()
{
	u16 i;
	for(i=0; i<20; i++)
	{
		Write_DAC(i);
		delay_10us(25);	//250us*20=5ms
	}
	
	for(i=20; i>0; i--)
	{
		Write_DAC(20);
		delay(5);
	}
	
	for(i=20; i>0; i--)
	{
		Write_DAC(i);
		delay_10us(25);
	}
	
	Write_DAC(0);
	
	delay(10);
}

//产生锯齿波
void generate_Sawtooth()
{
	u16 i;
	for(i=0; i<20; i++)
	{
		Write_DAC(i);
		delay(1);
	}
	Write_DAC(0);
}

//产生正弦波
uchar code sin_table[] = {}

void generate_sin()
{
    uint value;

    value = sin_table[index]    //这里可以*amplitude来控制幅值
    Write_DAC(value);
    delay_10us();   
    index++;    //因为index是unsigned char(0-255)，溢出后自动回0，正好对应256个点
}
````

其他说明
````c
1.sin_table值的获取
    1.Excel中A列输入0~255
    2.B1 = SIN(2*PI()*A1/256)
    3.C1 = INT(128+127*B1)
    4.D1 = DEC2HEX(C1,2)
    5.E1 = "0X"&D1
    6.粘贴到word（只保留原文本）
    7.替换：^p -> ,

````