.. _esp8266_quickref:

ESP8266 快速参考手册
===============================
.. only:: not latex

   .. image:: http://www.01studio.org/micropython/picture/pyWiFi-ESP8266_pinout.png
      :alt: pyWiFi-ESP8266 pinout
      :width: 700px

.. only:: latex

   .. image:: http://www.01studio.org/micropython/picture/pyWiFi-ESP8266_pinout.png
      :alt: pyWiFi-ESP8266 pinout

ESP8266开发板（图片来源：01Studio）

以下是快速参考内容.  如果你是第一次使用ESP8266开发板，请考虑先阅读以下章节内容:

.. toctree::
   :maxdepth: 1

   general.rst
   tutorial/index.rst

安装 MicroPython
----------------------

请参考教程的相应部分: :ref:`intro`. 它还包括故障排除小节.

通用控制
---------------------

MicroPython 的串口交互调试（REPL）在 UART0 (GPIO1=TX, GPIO3=RX)，波特率为：115200。
Tab按键补全功能对于找到每个对象的使用方法非常有用。
粘贴模式 (ctrl-E) 对需要复制比较多的python代码到REPL非常有用。

The :mod:`machine` module::

    import machine

    machine.freq()          # 获取CPU当前工作频率
    machine.freq(160000000) # 设置CPU的工作频率为 160 MHz

The :mod:`esp` module::

    import esp

    esp.osdebug(None)       # 关闭原厂 O/S 调试信息
    esp.osdebug(0)          # 将原厂 O/S 调试信息重定向到 UART(0) 输出

Networking
----------

The :mod:`network` module::

    import network

    wlan = network.WLAN(network.STA_IF) # 创建station接口
    wlan.active(True)       # 激活接口
    wlan.scan()             # 搜索允许的访问SSID
    wlan.isconnected()      # 检查创建的station是否连接到AP
    wlan.connect('essid', 'password') # 连接到指定ESSID网络
    wlan.config('mac')      # 获取接口的MAC地址
    wlan.ifconfig()         # 获取接口的 IP/netmask(子网掩码)/gw(网关)/DNS 地址

    ap = network.WLAN(network.AP_IF) # 创捷一个AP热点接口
    ap.active(True)         # 激活接口
    ap.config(essid='ESP-AP') # 设置AP的ESSID名称
    
连接到本地WIFI网络的函数参考::

    def do_connect():
        import network
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            print('connecting to network...')
            wlan.connect('essid', 'password')
            while not wlan.isconnected():
                pass
        print('network config:', wlan.ifconfig())

一旦网络建立成功，就可以通过 :mod:`socket <usocket>` 模块创捷和使用 TCP/UDP socket通讯。 

延时和时间
------------

Use the :mod:`time <utime>` module::

    import time

    time.sleep(1)           # 睡眠1秒
    time.sleep_ms(500)      # 睡眠500毫秒
    time.sleep_us(10)       # 睡眠10微妙
    start = time.ticks_ms() # 获取毫秒计时器开始值
    delta = time.ticks_diff(time.ticks_ms(), start) # 计算从开始到当前时间的差值

定时器
------

支持虚拟 (基于RTOS) 定时器。使用 :ref:`machine.Timer <machine.Timer>` 模块通过设置 timer ID 号为 -1::

    from machine import Timer

    tim = Timer(-1) 
    tim.init(period=5000, mode=Timer.ONE_SHOT, callback=lambda t:print(1)) #1次
    tim.init(period=2000, mode=Timer.PERIODIC, callback=lambda t:print(2)) #周期

该周期的单位为毫秒(ms)。

引脚和GPIO口
-------------

使用 :ref:`machine.Pin <machine.Pin>` 模块::

    from machine import Pin

    p0 = Pin(0, Pin.OUT)    # 创建对象p0，对应GPIO0口输出
    p0.on()                 # 设置引脚为 "on" (1)高电平 
    p0.off()                # 设置引脚为 "off" (0)低电平
    p0.value(1)             # 设置引脚为 "on" (1)高电平 

    p2 = Pin(2, Pin.IN)     # 创建对象p2，对应GPIO2口输入
    print(p2.value())       # 获取引脚输入值, 0（低电平） or 1（高电平）

    p4 = Pin(4, Pin.IN, Pin.PULL_UP) # 打开内部上拉电阻
    p5 = Pin(5, Pin.OUT, value=1) # 初始化时候设置引脚的值为 1（高电平）

以下为可用引脚: 0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 分别对应ESP8266芯片的实际GPIO引脚编号。
请注意，很多用户使用自己的开发板有特定的引脚命名方式 (例如： D0, D1, ...)。由于MicroPython致力于支持
不同的开发板和模块，因此我们采用最原始简单具且有共同特征的引脚命名方式。如果你使用自己的开发板，请参考其原理图。

请注意，引脚 Pin(1) and Pin(3) 是串口交互（REPL） UART TX 和 RX 引脚.
同时请注意 Pin(16) 是一个特殊的引脚 (用于从深度睡眠模式中唤醒) 所以有可能不能使用高级的类模块如``Neopixel``.


UART (serial bus)
-----------------

See :ref:`machine.UART <machine.UART>`. ::

    from machine import UART
    uart = UART(0, baudrate=9600)
    uart.write('hello')
    uart.read(5) # read up to 5 bytes

Two UARTs are available. UART0 is on Pins 1 (TX) and 3 (RX). UART0 is
bidirectional, and by default is used for the REPL. UART1 is on Pins 2
(TX) and 8 (RX) however Pin 8 is used to connect the flash chip, so
UART1 is TX only.

When UART0 is attached to the REPL, all incoming chars on UART(0) go
straight to stdin so uart.read() will always return None.  Use
sys.stdin.read() if it's needed to read characters from the UART(0)
while it's also used for the REPL (or detach, read, then reattach).
When detached the UART(0) can be used for other purposes.

If there are no objects in any of the dupterm slots when the REPL is
started (on hard or soft reset) then UART(0) is automatically attached.
Without this, the only way to recover a board without a REPL would be to
completely erase and reflash (which would install the default boot.py which
attaches the REPL).

To detach the REPL from UART0, use::

    import uos
    uos.dupterm(None, 1)

The REPL is attached by default. If you have detached it, to reattach
it use::

    import uos, machine
    uart = machine.UART(0, 115200)
    uos.dupterm(uart, 1)


PWM (脉宽调制)
----------------------------

PWM 可以通过所有引脚输出除了 Pin(16).  所有通道都有1个特定的频率，从1到1000之间（单位是Hz）。占空比的值为0至1023之间。

Use the ``machine.PWM`` class::

    from machine import Pin, PWM

    pwm0 = PWM(Pin(0))      # 从1个引脚中创建 PWM 对象
    pwm0.freq()             # 获取当前频率
    pwm0.freq(1000)         # 设置频率
    pwm0.duty()             # 获取当前占空比
    pwm0.duty(200)          # 设置占空比
    pwm0.deinit()           # 关闭引脚的 PWM

    pwm2 = PWM(Pin(2), freq=500, duty=512) # 在同一语句下创建和配置 PWM

ADC (模数转换)
----------------------------------

ADC 需要使用专用的引脚。请注意ADC引脚输入电压必须是0v 至 1.0v。

Use the :ref:`machine.ADC <machine.ADC>` class::

    from machine import ADC

    adc = ADC(0)            # 在ADC引脚上创建ADC对象
    adc.read()              # 读取测量值, 0-1024

软件SPI总线
----------------

EPS32内部有两个SPI驱动。其中1个是通过软件实现 (bit-banging)，并允许配置到所有
引脚， 通过 :ref:`machine.SPI <machine.SoftSPI>` 类模块配置::

    from machine import Pin, SoftSPI

    # 在给定的引脚上创建SPI总线
    # polarity（极性）是指 SCK 空闲时候的状态
    # phase=0 （相位）表示SCK在第1个边沿开始取样，phase=1 表示在第2个边沿开始。
    spi = SoftSPI(-1, baudrate=100000, polarity=1, phase=0, sck=Pin(0), mosi=Pin(2), miso=Pin(4))

    spi.init(baudrate=200000) # 设置频率

    spi.read(10)            # 在MISO引脚读取10字节数据
    spi.read(10, 0xff)      # 在MISO引脚读取10字节数据同时在MOSI输出0xff

    buf = bytearray(50)     # 建立缓冲区
    spi.readinto(buf)       # 读取数据并存放在缓冲区 (这里读取50个字节)
    spi.readinto(buf, 0xff) # 读取数据并存放在缓冲区，同时在MOSI输出0xff

    spi.write(b'12345')     # 在MOSI引脚上写5字节数据

    buf = bytearray(4)      # 建立缓冲区
    spi.write_readinto(b'1234', buf) # 在MOSI引脚上写数据并将MISO读取数据存放到缓冲区
    spi.write_readinto(buf, buf) # 在MOSI引脚上写缓冲区的数据并将MISO读取数据存放到缓冲区


硬件SPI总线
----------------

有两个硬件SPI通道允许更高速率传输（到达80MHz）。只可以运行在以下对应引脚:
``MISO`` 是 GPIO12, ``MOSI`` 是 GPIO13, 以及 ``SCK`` 是 GPIO14. 除了引脚参数配置外，
使用方法和上面软件SPI总线一样（因为引脚是固定的） ::

    from machine import Pin, SPI

    hspi = SPI(1, baudrate=80000000, polarity=0, phase=0)

(``SPI(0)`` 用在内部flash上，不对用户开放使用。)

I2C总线
-------

The I2C driver is implemented in software and works on all pins,
and is accessed via the :ref:`machine.I2C <machine.I2C>` class (which is an
alias of :ref:`machine.SoftI2C <machine.SoftI2C>`)::


    from machine import Pin, I2C

    # construct an I2C bus
    i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)

    i2c.readfrom(0x3a, 4)   # 从地址为0x3a的从机设备读取4字节数据
    i2c.writeto(0x3a, '12') # 向地址为0x3a的从机设备写入数据"12" 

    buf = bytearray(10)     # 创建1个10字节缓冲区
    i2c.writeto(0x3a, buf)  # 写入缓冲区数据到从机

实时时钟 (RTC)
---------------------

See :ref:`machine.RTC <machine.RTC>` ::

    from machine import RTC

    rtc = RTC()
    rtc.datetime((2017, 8, 23, 1, 12, 48, 0, 0)) # 设置时间（年，月，日，星期，时，分，秒，微秒）
                                                 # 其中星期使用0-6表示星期一至星期日。                                                  
    rtc.datetime() # 获取当前日期和时间

    # synchronize with ntp
    # need to be connected to wifi
    import ntptime
    ntptime.settime() # set the rtc datetime from the remote server
    rtc.datetime()    # get the date and time in UTC

.. note:: Not all methods are implemented: `RTC.now()`, `RTC.irq(handler=*) <RTC.irq>`
          (using a custom handler), `RTC.init()` and `RTC.deinit()` are
          currently not supported.


深度睡眠模式
---------------

将GPIO16引脚连接到复位（HUZZAH的RST）。那么下面的代码可以用来
睡眠、唤醒和检测复位唤醒::

    import machine

    # 配置 RTC.ALARM0 用于定时唤醒设备
    rtc = machine.RTC()
    rtc.irq(trigger=rtc.ALARM0, wake=machine.DEEPSLEEP)

    # 检测设备是否通过深度睡眠GPIO16引脚唤醒
    if machine.reset_cause() == machine.DEEPSLEEP_RESET:
        print('woke from a deep sleep')

    # 设置 RTC.ALARM0 定时器时间为10秒（唤醒设备）
    rtc.alarm(rtc.ALARM0, 10000)

    # 让设备进入睡眠状态
    machine.deepsleep()

单总线驱动（OneWire）
---------------------

单总线驱动允许通过软件在各个引脚上实现::

    from machine import Pin
    import onewire

    ow = onewire.OneWire(Pin(12)) # 在引脚 GPIO12 创建单总线对象ow
    ow.scan()               # 扫描设备
    ow.reset()              # 复位
    ow.readbyte()           # 读取1个字节
    ow.writebyte(0x12)      # 写入1个字节
    ow.write('123')         # 写入多个字节
    ow.select_rom(b'12345678') # 根据ROM编号选择总线上的指定设备

下面是一个DS18B20设备的驱动函数::

    import time, ds18x20
    ds = ds18x20.DS18X20(ow)
    roms = ds.scan()
    ds.convert_temp()
    time.sleep_ms(750)
    for rom in roms:
        print(ds.read_temp(rom))

确保数据引脚连接了 4.7k 的上拉电阻。另外请注意每次采集温度都需要用到
``convert_temp()`` 模块.

NeoPixel 彩灯驱动
------------------

Use the ``neopixel`` module::

    from machine import Pin
    from neopixel import NeoPixel

    pin = Pin(0, Pin.OUT)   # 设置引脚GPIO0来驱动 NeoPixels
    np = NeoPixel(pin, 8)   # 在GPIO0上创建一个 NeoPixel对象，包含8个灯珠
    np[0] = (255, 255, 255) # 设置第一个灯珠显示数据为白色
    np.write()              # 写入数据
    r, g, b = np[0]         # 获取第一个灯珠的颜色

低级别的 NeoPixel 驱动::

    import esp
    esp.neopixel_write(pin, grb_buf, is800khz)

.. Warning::
   By default ``NeoPixel`` is configured to control the more popular *800kHz*
   units. It is possible to use alternative timing to control other (typically
   400kHz) devices by passing ``timing=0`` when constructing the
   ``NeoPixel`` object.

APA102 驱动
-------------

Use the ``apa102`` module::

    from machine import Pin
    from apa102 import APA102

    clock = Pin(14, Pin.OUT)     # set GPIO14 to output to drive the clock
    data = Pin(13, Pin.OUT)      # set GPIO13 to output to drive the data
    apa = APA102(clock, data, 8) # create APA102 driver on the clock and the data pin for 8 pixels
    apa[0] = (255, 255, 255, 31) # set the first pixel to white with a maximum brightness of 31
    apa.write()                  # write data to all pixels
    r, g, b, brightness = apa[0] # get first pixel colour

For low-level driving of an APA102::

    import esp
    esp.apa102_write(clock_pin, data_pin, rgbi_buf)

DHT 驱动
----------

DHT 温湿度驱动允许通过软件在各个引脚上实现::

    import dht
    import machine

    d = dht.DHT11(machine.Pin(4))
    d.measure()
    d.temperature() # eg. 23 (°C)
    d.humidity()    # eg. 41 (% RH)

    d = dht.DHT22(machine.Pin(4))
    d.measure()
    d.temperature() # eg. 23.6 (°C)
    d.humidity()    # eg. 41.3 (% RH)

WebREPL (Web浏览器交互提示)
----------------------------------------

WebREPL (通过WebSockets的REPL, 可以通过浏览器使用) 是ESP8266端口实验的功能。
可以从 https://github.com/micropython/webrepl 下载并打开html文件运行。
(在线托管版可以通过访问 http://micropython.org/webrepl)直接使用, 通过执行以下
命令进行配置::

    import webrepl_setup

按照屏幕的提示操作。重启后，允许使用WebREPL。如果你禁用了开机自动启动WebREPL,可以
通过以下命令使用::

    import webrepl
    webrepl.start()

这个 WebREPL 通过连接到ESP8266的AP使用,如果你的路由器配网络配置正确，这个功能
也可以通过STA方式使用，那意味着你可以同时上网和调试ESP8266。(如果遇到不可行的特殊情况，
请先使用ESP8266 AP方式)。

除了终端/命令符的访问方式, WebREPL同时允许传输文件 (包含上传和下载)。Web客户端有相应的
功能按钮，也可以通过 ``webrepl_cli.py`` 模块上存储的命令行进行操作。

有关将文件传输到ESP8266其他支持的替代方法，请参阅MicroPython论坛。
