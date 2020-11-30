:mod:`ubluetooth` --- 低功耗蓝牙
=========================================

.. module:: ubluetooth
   :synopsis: 无线低功耗蓝牙功能

This module provides an interface to a Bluetooth controller on a board.
Currently this supports Bluetooth Low Energy (BLE) in Central, Peripheral,
Broadcaster, and Observer roles, as well as GATT Server and Client. A device
may operate in multiple roles concurrently.

.. note:: This module is still under development and its classes, functions,
          methods and constants are subject to change.

BLE 类
---------

函数
-----------

.. class:: BLE()

    返回单个BLE对象。

配置
-------------

.. method:: BLE.active([active], /)

    无线低功耗蓝牙（BLE）的状态（可选项）, 返回当前状态。

    在使用其他任何一个库之前必须开启无线低功耗蓝牙（BLE）。

.. method:: BLE.config('param', /)
            BLE.config(*, param=value, ...)

    Get or set configuration values of the BLE interface.  To get a value the
    parameter name should be quoted as a string, and just one parameter is
    queried at a time.  To set values use the keyword syntax, and one ore more
    parameter can be set at a time.



    通过*name*获取一个配置值. 现支持以下值:

    - ``'mac'``: The current address in use, depending on the current address mode.
      This returns a tuple of ``(addr_type, addr)``.

      See :meth:`gatts_write <BLE.gap_scan>` for details about address type.

      This may only be queried while the interface is currently active.

    - ``'addr_mode'``: Sets the address mode. Values can be:

        * 0x00 - PUBLIC - Use the controller's public address.
        * 0x01 - RANDOM - Use a generated static address.
        * 0x02 - RPA - Use resolvable private addresses.
        * 0x03 - NRPA - Use non-resolvable private addresses.

      By default the interface mode will use a PUBLIC address if available, otherwise
      it will use a RANDOM address.

    - ``'gap_name'``: Get/set the GAP device name used by service 0x1800,
      characteristic 0x2a00.  This can be set at any time and changed multiple
      times.

    - ``'rxbuf'``: Get/set the size in bytes of the internal buffer used to store
      incoming events.  This buffer is global to the entire BLE driver and so
      handles incoming data for all events, including all characteristics.
      Increasing this allows better handling of bursty incoming data (for
      example scan results) and the ability to receive larger characteristic values.

    - ``'mtu'``: Get/set the MTU that will be used during an MTU exchange. The
      resulting MTU will be the minimum of this and the remote device's MTU.
      MTU exchange will not happen automatically (unless the remote device initiates
      it), and must be manually initiated with
      :meth:`gattc_exchange_mtu<BLE.gattc_exchange_mtu>`.
      Use the ``_IRQ_MTU_EXCHANGED`` event to discover the MTU for a given connection.

事件处理
--------------

.. method:: BLE.irq(handler, /)

    注册一个BLE栈回调事件. *handler*需要两个对象, ``event`` (执行的函数)和``data``
    (特定事件的一组值).

    *trigger*（可选项）让你设置你的程序的目标事件。 默认设置为所有事件。


    **Note:** As an optimisation to prevent unnecessary allocations, the ``addr``,
    ``adv_data``, ``char_data``, ``notify_data``, and ``uuid`` entries in the
    tuples are read-only memoryview instances pointing to ubluetooth's internal
    ringbuffer, and are only valid during the invocation of the IRQ handler
    function.  If your program needs to save one of these values to access after
    the IRQ handler has returned (e.g. by saving it in a class instance or global
    variable), then it needs to take a copy of the data, either by using ``bytes()``
    or ``bluetooth.UUID()``, like this::

        connected_addr = bytes(addr)  # equivalently: adv_data, char_data, or notify_data
        matched_uuid = bluetooth.UUID(uuid)

    For example, the IRQ handler for a scan result might inspect the ``adv_data``
    to decide if it's the correct device, and only then copy the address data to be
    used elsewhere in the program.  And to print data from within the IRQ handler,
    ``print(bytes(addr))`` will be needed.


    An event handler showing all possible events::

    一个事件处理展示所有可能的事件::

        def bt_irq(event, data):
            if event == _IRQ_CENTRAL_CONNECT:
                # 一个中央单元连接到这个外围设备
                conn_handle, addr_type, addr = data
            elif event == _IRQ_CENTRAL_DISCONNECT:
                # 一个中央单元与这个外围设备断开链接
                conn_handle, addr_type, addr = data
            elif event == _IRQ_GATTS_WRITE:
                # 中央单元已写入此特征或描述符
                conn_handle, attr_handle = data
            elif event == _IRQ_GATTS_READ_REQUEST:
                # 中央单元发布消息，这是一个硬中断请求
                # 返回None值以拒绝读取
                # 请注意：该事件不支持ESP32
                conn_handle, attr_handle = data
            elif event == _IRQ_SCAN_RESULT:
                # 单次扫描结果
                addr_type, addr, adv_type, rssi, adv_data = data
            elif event == _IRQ_SCAN_DONE:
                # 扫描过程完成或者被手动停止
                pass
            elif event == _IRQ_PERIPHERAL_CONNECT:
                # 一个成功的gap_connect()
                conn_handle, addr_type, addr = data
            elif event == _IRQ_PERIPHERAL_DISCONNECT:
                # 与已连接的外围设备断开
                conn_handle, addr_type, addr = data
            elif event == _IRQ_GATTC_SERVICE_RESULT:
                # 为gattc_discover_services()找到的每个服务调用
                conn_handle, start_handle, end_handle, uuid = data
            elif event == _IRQ_GATTC_SERVICE_DONE:
                # Called once service discovery is complete.
                # Note: Status will be zero on success, implementation-specific value otherwise.
                conn_handle, status = data
            elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
                # 为gattc_discover_services()找到的每个特征调用
                conn_handle, def_handle, value_handle, properties, uuid = data
            elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
                # Called once service discovery is complete.
                # Note: Status will be zero on success, implementation-specific value otherwise.
                conn_handle, status = data
            elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
                # 为gattc_discover_descriptors()找到的每个描述符调用
                conn_handle, dsc_handle, uuid = data
            elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
                # Called once service discovery is complete.
                # Note: Status will be zero on success, implementation-specific value otherwise.
                conn_handle, status = data
            elif event == _IRQ_GATTC_READ_RESULT:
                # 已完成的gattc_read()
                conn_handle, value_handle, char_data = data
            elif event == _IRQ_GATTC_READ_DONE:
                # A gattc_read() has completed.
                # Note: The value_handle will be zero on btstack (but present on NimBLE).
                # Note: Status will be zero on success, implementation-specific value otherwise.
                conn_handle, value_handle, status = data
            elif event == _IRQ_GATTC_WRITE_DONE:
                # A gattc_write() has completed.
                # Note: The value_handle will be zero on btstack (but present on NimBLE).
                # Note: Status will be zero on success, implementation-specific value otherwise.               
                conn_handle, value_handle, status = data
            elif event == _IRQ_GATTC_NOTIFY:
                # 外部设备已发送通知请求
                conn_handle, value_handle, notify_data = data
            elif event == _IRQ_GATTC_INDICATE:
                # 外部设备已发送指示请求
                conn_handle, value_handle, notify_data = data
            elif event == _IRQ_GATTS_INDICATE_DONE:
                # A client has acknowledged the indication.
                # Note: Status will be zero on successful acknowledgment, implementation-specific value otherwise.
                conn_handle, value_handle, status = data
            elif event == _IRQ_MTU_EXCHANGED:
                # MTU exchange complete (either initiated by us or the remote device).
                conn_handle, mtu = data

以下是事件码::

    from micropython import const
    _IRQ_CENTRAL_CONNECT = const(1)
    _IRQ_CENTRAL_DISCONNECT = const(2)
    _IRQ_GATTS_WRITE = const(3)
    _IRQ_GATTS_READ_REQUEST = const(4)
    _IRQ_SCAN_RESULT = const(5)
    _IRQ_SCAN_DONE = const(6)
    _IRQ_PERIPHERAL_CONNECT = const(7)
    _IRQ_PERIPHERAL_DISCONNECT = const(8)
    _IRQ_GATTC_SERVICE_RESULT = const(9)
    _IRQ_GATTC_SERVICE_DONE = const(10)
    _IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
    _IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
    _IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
    _IRQ_GATTC_DESCRIPTOR_DONE = const(14)
    _IRQ_GATTC_READ_RESULT = const(15)
    _IRQ_GATTC_READ_DONE = const(16)
    _IRQ_GATTC_WRITE_DONE = const(17)
    _IRQ_GATTC_NOTIFY = const(18)
    _IRQ_GATTC_INDICATE = const(19)
    _IRQ_GATTS_INDICATE_DONE = const(20)
    _IRQ_MTU_EXCHANGED = const(21)

为了节省固件的空间, 这些内容没有包含在:mod:`ubluetooth` 模块，需自行从上面的列表中选择你所
需要的事件码到你的程序中。


广播规则（对外宣传者）
-----------------------------

.. method:: BLE.gap_advertise(interval_us, adv_data=None, *, resp_data=None, connectable=True)

    在指定时间间隔开始广播(**微**\ 秒). 该间隔将会精确到625us（微妙）。
    将*interval_us*设为``None``以停止广播。

    *adv_data*和*resp_data*可以是任何可实现的缓冲协议 (例如``bytes``, ``bytearray``, ``str``)。
    *adv_data*包含于任何广播里, *resp_data* 是对已激活的扫描仪的回复。

    说明：如果*adv_data* (或*resp_data*)是``None``, 接下来传递给上一个对``gap_advertise``调用
    的值将会被再次使用。
    This allows a broadcaster to resume advertising 这就意味着只要``gap_advertise(interval_us)``
    就可以让广播恢复对外宣传.
    提供一个空的``bytes``以清除广播负载，例如``b''``.


观察者角色（扫描仪）
-----------------------

.. method:: BLE.gap_scan(duration_ms, interval_us=1280000, window_us=11250, active=False, /)

    持续扫描一段时间(**毫**\ 秒)。

    如果要让开发版一直扫描，请将*duration_ms*设置为``0``.

    如果要停止扫描，将*duration_ms*设置为``None``.

    扫描仪将每*interval_us*微秒运行*window_us*微秒，共*duration_ms*毫秒。
    默认选项分别为 1.28 秒和11.25毫秒(后台扫描).


    For each scan result the ``_IRQ_SCAN_RESULT`` event will be raised, with event
    data ``(addr_type, addr, adv_type, rssi, adv_data)``.

    ``addr_type`` values indicate public or random addresses:
        * 0x00 - PUBLIC
        * 0x01 - RANDOM (either static, RPA, or NRPA, the type is encoded in the address itself)

    ``adv_type`` values correspond to the Bluetooth Specification:

        * 0x00 - ADV_IND - connectable and scannable undirected advertising
        * 0x01 - ADV_DIRECT_IND - connectable directed advertising
        * 0x02 - ADV_SCAN_IND - scannable undirected advertising
        * 0x03 - ADV_NONCONN_IND - non-connectable undirected advertising
        * 0x04 - SCAN_RSP - scan response

    当扫描停止时(由于扫描过程结束或者明确地停止)，会引发``_IRQ_SCAN_COMPLETE``事件

    ``active`` can be set ``True`` if you want to receive scan responses in the results.

    When scanning is stopped (either due to the duration finishing or when
    explicitly stopped), the ``_IRQ_SCAN_DONE`` event will be raised.


Central Role
------------

A central device can connect to peripherals that it has discovered using the observer role (see :meth:`gap_scan<BLE.gap_scan>`) or with a known address.

.. method:: BLE.gap_connect(addr_type, addr, scan_duration_ms=2000, /)

    Connect to a peripheral.

    See :meth:`gap_scan <BLE.gap_scan>` for details about address types.

    On success, the ``_IRQ_PERIPHERAL_CONNECT`` event will be raised.


Peripheral Role
---------------

A peripheral device is expected to send connectable advertisements (see
:meth:`gap_advertise<BLE.gap_advertise>`). It will usually be acting as a GATT
server, having first registered services and characteristics using
:meth:`gatts_register_services<BLE.gatts_register_services>`.

When a central connects, the ``_IRQ_CENTRAL_CONNECT`` event will be raised.


Central & Peripheral Roles
--------------------------

.. method:: BLE.gap_disconnect(conn_handle, /)

    Disconnect the specified connection handle. This can either be a
    central that has connected to this device (if acting as a peripheral)
    or a peripheral that was previously connected to by this device (if acting
    as a central).

    On success, the ``_IRQ_PERIPHERAL_DISCONNECT`` or ``_IRQ_CENTRAL_DISCONNECT``
    event will be raised.

    Returns ``False`` if the connection handle wasn't connected, and ``True``
    otherwise.

外围角色 (GATT服务器)
-----------------------------

一个蓝牙外设已经有一套已经注册好的服务。每个服务可能包含带有一个值的特征，
特征也包含了自带值的描述符。

这些值都存储在本地，并由服务注册期间生成的“值句柄”访问。他们也被远程中央设备读取
或写入。另外，外围设备可以通过连接句柄将特征“通知”到连接的中央设备。

特征和描述有一个20字节的缺省最大值。
任何通过中央单元写入的内容将会被截断到此长度。然而，
任何一次本地写入将会增加最大尺寸，因此如果你想允许从中心向给定特征进行更大的写入,
在注册之后请使用:meth:`gatts_write<BLE.gatts_write>`。
例如：``gatts_write(char_handle, bytes(100))``

.. method:: BLE.gatts_register_services(services_definition, /)

    Configures the server with the specified services, replacing any
    existing services.


    *services_definition*是一个**服务**列表，每一个**服务**是一个包含两
    个元素的元组，包含了一个UUID和一整个列表的**特征**。

    每个**特征** 是一个由两到三个元素的元组t，包含一个UUID，一个**flags**值，
    以及可选的*descriptors*列表。

    每个**描述符**是一个包含两个元素的元组，包含一个UUID和一个**flags**。

    The **flags** are a bitwise-OR combination of the
    :data:`ubluetooth.FLAG_READ`, :data:`ubluetooth.FLAG_WRITE` and
    :data:`ubluetooth.FLAG_NOTIFY` values defined below.

    返回的是是一列表(一个服务一个元素)的元组(每一个元素有值句柄). 
    特征和描述符句柄按定义顺序展平到同一元组中。

    下面的示例注册了两个服务(心跳和Nordic通用异步收发器)::

        HR_UUID = bluetooth.UUID(0x180D)
        HR_CHAR = (bluetooth.UUID(0x2A37), bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY,)
        HR_SERVICE = (HR_UUID, (HR_CHAR,),)
        UART_UUID = bluetooth.UUID('6E400001-B5A3-F393-E0A9-E50E24DCCA9E')
        UART_TX = (bluetooth.UUID('6E400003-B5A3-F393-E0A9-E50E24DCCA9E'), bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY,)
        UART_RX = (bluetooth.UUID('6E400002-B5A3-F393-E0A9-E50E24DCCA9E'), bluetooth.FLAG_WRITE,)
        UART_SERVICE = (UART_UUID, (UART_TX, UART_RX,),)
        SERVICES = (HR_SERVICE, UART_SERVICE,)
        ( (hr,), (tx, rx,), ) = bt.gatts_register_services(SERVICES)

    这里有三个值句柄(``hr``, ``tx``, ``rx``)可用于:meth:`gatts_read <BLE.gatts_read>`, 
    :meth:`gatts_read <BLE.gatts_read>`, :meth:`gatts_write <BLE.gatts_write>`, :meth:`gatts_notify <BLE.gatts_notify>`, and
    :meth:`gatts_indicate <BLE.gatts_indicate>`.

    **提示：** 注册服务前对外显示必须被停止。

.. method:: BLE.gatts_read(value_handle, /)

    读取此句柄的本地值(要么是由:meth:`gatts_write <BLE.gatts_write>`编写的，
    要么是由远程中央单元编写的).

.. method:: BLE.gatts_write(value_handle, data, /)


    写入此句柄的本地值，该值可以被中央单元读取。

.. method:: BLE.gatts_notify(conn_handle, value_handle, data=None, /)

    Sends a notification request to a connected client.

    If *data* is not ``None``, then that value is sent to the client as part of
    the notification. The local value will not be modified.

    Otherwise, if *data* is ``None``, then the current local value (as
    set with :meth:`gatts_write <BLE.gatts_write>`) will be sent.

.. method:: BLE.gatts_indicate(conn_handle, value_handle, /)

    Sends an indication request to a connected client.

    **Note:** This does not currently support sending a custom value, it will
    always send the current local value (as set with :meth:`gatts_write
    <BLE.gatts_write>`).

    On acknowledgment (or failure, e.g. timeout), the
    ``_IRQ_GATTS_INDICATE_DONE`` event will be raised.

.. method:: BLE.gatts_set_buffer(value_handle, len, append=False, /)

    设置以字节为单位的值的内部缓冲区大小。这将限制可接收的最大可能写操作。
    默认值是20。

    将*append*设为``True`` 将所有远程写入追加，而不是替换，当前的值。
    大部分*len*字节可以通过这个方式缓冲。
    当你使用:meth:`gatts_read <BLE.gatts_read>`，这个值将在读取后被清除。
    此功能在实现Nordic UART服务时非常有用。

GATT Client
-----------

中央处理规则(GATT客户端)
--------------------------

.. method:: BLE.gap_connect(addr_type, addr, scan_duration_ms=2000, /)

    连接到外设。

    连接成功后, 将引发``_IRQ_PERIPHERAL_CONNECT``事件。

.. method:: BLE.gap_disconnect(conn_handle)

    断开指定的连接句柄。

    断开成功后，将引发``_IRQ_PERIPHERAL_DISCONNECT``事件。

    如果链接句柄，返回``False``，``True``则相反。

.. method:: BLE.gattc_discover_services(conn_handle, [uuid])

    查询连接的外围设备以获取其服务。

A GATT client can discover and read/write characteristics on a remote GATT server.

It is more common for a central role device to act as the GATT client, however
it's also possible for a peripheral to act as a client in order to discover
information about the central that has connected to it (e.g. to read the
device name from the device information service).

.. method:: BLE.gattc_discover_services(conn_handle, uuid=None, /)

    Query a connected server for its services.

    Optionally specify a service *uuid* to query for that service only.

    For each service discovered, the ``_IRQ_GATTC_SERVICE_RESULT`` event will
    be raised, followed by ``_IRQ_GATTC_SERVICE_DONE`` on completion.

.. method:: BLE.gattc_discover_characteristics(conn_handle, start_handle, end_handle, uuid=None, /)

    查询连接的外围设备以获取指定范围内的特征。

    Optionally specify a characteristic *uuid* to query for that
    characteristic only.

    You can use ``start_handle=1``, ``end_handle=0xffff`` to search for a
    characteristic in any service.

    For each characteristic discovered, the ``_IRQ_GATTC_CHARACTERISTIC_RESULT``
    event will be raised, followed by ``_IRQ_GATTC_CHARACTERISTIC_DONE`` on completion.

.. method:: BLE.gattc_discover_descriptors(conn_handle, start_handle, end_handle, /)

    查询连接的外设以查找指定范围内的描述符。

    For each descriptor discovered, the ``_IRQ_GATTC_DESCRIPTOR_RESULT`` event
    will be raised, followed by ``_IRQ_GATTC_DESCRIPTOR_DONE`` on completion.

.. method:: BLE.gattc_read(conn_handle, value_handle, /)

    查询连接的外设以查找指定范围内的描述符。对连接的外设发出远程读取以
    获取指定的特征或描述符句柄。

    When a value is available, the ``_IRQ_GATTC_READ_RESULT`` event will be
    raised. Additionally, the ``_IRQ_GATTC_READ_DONE`` will be raised.

.. method:: BLE.gattc_write(conn_handle, value_handle, data, mode=0, /)

    为指定的特征或描述符句柄向连接的外围设备发出远程写入。

    The argument *mode* specifies the write behaviour, with the currently
    supported values being:

        * ``mode=0`` (default) is a write-without-response: the write will
          be sent to the remote server but no confirmation will be
          returned, and no event will be raised.
        * ``mode=1`` is a write-with-response: the remote server is
          requested to send a response/acknowledgement that it received the
          data.

    If a response is received from the remote server the
    ``_IRQ_GATTC_WRITE_DONE`` event will be raised.

.. method:: BLE.gattc_exchange_mtu(conn_handle, /)

    Initiate MTU exchange with a connected server, using the preferred MTU
    set using ``BLE.config(mtu=value)``.

    The ``_IRQ_MTU_EXCHANGED`` event will be raised when MTU exchange
    completes.

    **Note:** MTU exchange is typically initiated by the central. When using
    the BlueKitchen stack in the central role, it does not support a remote
    peripheral initiating the MTU exchange. NimBLE works for both roles.



class UUID
----------


Constructor
-----------

.. class:: UUID(value, /)

    创建具有指定**值**的UUID实例。

    **值**可以是：

    - 16位整数 例如 ``0x2908``.
    - 128位UUID字符串 例如 ``'6E400001-B5A3-F393-E0A9-E50E24DCCA9E'``.


常数
---------

.. data:: ubluetooth.FLAG_READ
          ubluetooth.FLAG_WRITE
          ubluetooth.FLAG_NOTIFY
