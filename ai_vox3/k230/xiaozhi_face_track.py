"""
xiaozhi_face_track.py —— K230 当小智的眼睛（日常运行版，代码精简）
干的事：摄像头抓帧 → 跑人脸检测 → 把人脸框归一化 → UDP 发给 AI-VOX3:3333
        AI-VOX3 那边的 CompanionHead 收到就驱动二轴云台跟随。
报文（与 companion_head.cc:311 的 sscanf 严格对应）：
    F,<cx>,<cy>,<w>,<h>,<conf>\n     "F," 后面是 5 个数字，0..1000 / conf 0..100
    N\n                              本帧没检测到人脸
相比 xiaozhi_link_main.py，这个版本砍掉了：
  · WebSocket 拍照服务（本固件没有 media.mjpeg，用不了，先不占空间）
  · K230 本地驱云台（云台交给小智驱动更好，见 24 号文档 §4）
好处是代码量减半，走串口 REPL 灌进去不容易被吞字节。
要存成开机自启：CanMV IDE → 工具 → Save open script to CanMV Board (as main.py)
"""
import gc
import os
import sys
import time
WIFI_SSID = "your-ssid"              # ★ v46：默认回 <ssid> 网；现场换网改 /sdcard/wifi.txt 第1行
WIFI_PASS = "REDACTED-WIFI-PASS"
VOX3_IP = ""                     # 仅 USE_BROADCAST=False 时填；v46 起默认走广播不用管
VOX3_PORT = 3333
USE_BROADCAST = True             # 广播——VOX3 是 DHCP 也无所谓，IP 不用猜
BROADCAST_IP = "255.255.255.255" # ★ v46：受限广播，任何网段都通（不用再改网段广播地址）
SEND_HZ = 25                     # [v52] 20→25：主控 20Hz 轮询，留余量别卡边界
NOFACE_HZ = 2                   # 没检测到时的发送频率（别刷屏）
HMIRROR = False                 # 翻转摄像头画面本身（显示 + 检测都用翻转后的图）
VMIRROR = False
REPORT_HMIRROR = True
REPORT_VMIRROR = False
SMOOTH_ALPHA = 0.90             # [v52] 0.82→0.90：滞后≈0.5 帧，肉眼就是"跟手"
MAX_FACE_W = 760                # [v53] 520->760：人凑近时脸占画面 2/3 别当误检丢掉
MAX_FACE_H = 660                # [v53] 420->660
MIN_FACE_W = 24                 # v48: 16->24, 挡小噪点(24‰=15px)
MIN_FACE_H = 50                 # v48 新增：纵向下限(50‰=24px)
MAX_FACE_H_LIMIT = 700          # [v53] 400->700（同上：允许近脸）
# ★ 2026-10-09：真人脸的框是"略瘦的长方形"，宽高比大约 0.7~0.95。
#   实测画面里混进来的误检是又高又瘦的（w/h≈0.55，例如 263x461、267x508），
#   或者干脆占了半个画面。用宽高比卡一道，比只看绝对尺寸准得多。
MIN_ASPECT = 0.45               # v48: 0.50->0.45, 本模型框系统性偏瘦(实测0.5~0.61)
MAX_ASPECT = 1.20               # v48: 2.00->1.20, 横向长条误检(桌沿/书本/相框)全挡
STABLE_FRAMES = 4               # ★ v45：两级判定（见下），单变量保留给旧逻辑兜底
STABLE_FRAMES_FIRST = 3         # ★v53 4->3：首锁再快一帧（≈0.2s）
                                #       （诊断 STAB 2/14、3/14 就是这么来的）。
                                #       误检已由 v49 的 box_ok() 挡掉，不必靠死等。
STABLE_FRAMES_TRACK = 3         # ★ v48：2->3（≈0.1s）
                                #   误检靠 aspect/尺寸/CONF_TH 三道闸挡，不靠死等
STABLE_DIST = 260               # ★v50 160->260：蹲下时 cy 每秒能变 200+，160 会一直清零
MAX_JUMP = 400                  # [v52] 240→400：横移 400‰/包≈9.6 屏宽/秒，够快
LOST_N_MS = 2600                # ★v50 1200->2600：短暂丢检别急着报 N，否则云台刚锁上就丢
ENABLE_CAM_WS = True
CAM_WS_PORT = 8080
JPEG_QUALITY = 60
JPEG_TARGET_BYTES = 60000
SHOT_FRAMES = 2
SHOT_TIMEOUT_MS = 8000
SHOT_INLINE = False            # ★ 2026-10-09 改回 False：True 会在出流时抓帧（两路取帧接口
                               #   混用必卡死，doc26 结论），实测拍照必崩。False=先 destroy
                               #   停流出再拍照，拍完 reset（慢 40s，但能成功）。
SHOT_JPEG = [None]
SHOT_JPEG_AT = [0]
SHOT_SMALL = [240, 240]
SHOT_JPEG_SMALL = [None]
SHOT_PUSHED = [0]
SHOT_CACHE_MS = 6000
SHOT_CMD_PORT = 3334            # 本板在这个 UDP 口等指令
SHOT_SERVE_MS = 20000           # 停流出后最多等多久小智来取图
HB_LINE = b"F,500,500,80,80,90\n"
MIRROR_TO_PC = True             # 把每条 F/N 报文**顺手**抄送一份到电脑，方便调角度
MIRROR_IP = ""                  # ★ v46：由 wifi.txt 第3行（PC_IP）填；空=不镜像
MIRROR_PORT = 3335
DISPLAY_EVERY = 3              # [v53] 2->3：显示再降一档，省下的时间给跟踪
                               #       640x480 OSD 合成+刷屏是单帧最大开销，
                               #       降到 1/2 后帧率明显上去，而人脸框
                               #       看起来仍是 10~12fps，够用。
                               #       设 1 = 恢复每帧刷新。
WD_TIMEOUT_MS = 6000           # [v52] 6 秒没跑成功一帧 -> machine.reset()
USE_HW_WDT = False             # [v52] 硬件看门狗：各版 CanMV 的 WDT(id, timeout)
                               #       签名不一致，参数被误解会变成"每 N 秒重启一次"。
                               #       先在 v52 里默认关闭，只保留软件看门狗（已能覆盖
                               #       "循环还在转但卡在取帧/显示"这类停摆）。
                               #       现场稳定后再改 True 试，出问题改回来即可。
DISPLAY_MODE = "lcd"            # lcd=板载屏 / ide=IDE虚拟显示(慢) / hdmi=外接
RGB888P_SIZE = [640, 480]       # 送进 AI 的分辨率
DISPLAY_SIZE = [640, 480]       # 板载屏 640x480
KMODEL_PATH = "/sdcard/kmodel/face_detection_320.kmodel"
ANCHORS_PATH = "/sdcard/utils/prior_data_320.bin"
MODEL_INPUT_SIZE = [320, 320]
CONF_TH = 0.4                   # ★ v45：0.5→0.4，远脸/侧脸别漏检（误检已有三道闸）
NMS_TH = 0.2
DEBUG = False                   # [v54] 关掉！串口是死的，stdout 写满会卡住主循环
LOG_TX = False
LOG_IP = ""                     # ★ v46：由 wifi.txt 第3行（PC_IP）填；空=不打日志到 PC
LOG_PORT = 3335
LOG_SOCK = [None]
SCRIPT_VER = "v54"              # v54: 见文件头 v54 说明
# ================= v54 改动 =================
#  ① ★ DEBUG 关掉。DEBUG=True 时 send_face() 每发一包就 print 一行，
#     25Hz 就是每秒 25 行往 stdout 灌。而这块板子的串口链路是死的
#     （COM33 既无输出也无回显），stdout 没人读、缓冲迟早写满 ——
#     print 一阻塞，整个跟踪循环就跟着停。这是 14fps 的第二个大嫌疑。
#     串口本来也看不到东西，关掉零损失；要看状态走 #s= 诊断行（走网络）。
#  ② 热重载真正修好。原来 HOT_RELOAD 查的是 PC_HOST:8123（PC_HOST 由
#     /sdcard/wifi.txt 第3行填，现场是空的 → 这段代码从来没执行过），
#     路径还是根目录的 /_version.txt，而 OTA 实际在
#     /k230-8f3c9a1e/_version.txt。现在改用独立的 OTA_* 常量直连
#     your.server.example:18123，60 秒查一次、超时 2 秒。
#     **从此改脚本只需改服务端 _version.txt，板上 1 分钟内自动重启拉新，
#     再也不用拔插 USB。**
#  ③ 诊断行开头带上脚本版本（#s=v54 ...），一眼能看出板上跑的是哪版 ——
#     v52/v53 两次排查都卡在"到底新代码生效没有"这个问题上。
# ================= v53 改动（性能为主） =================
# 用户反馈"还是有点慢，不精准"。v52 已解决"停摆"，现场实测 fps=14.2，
# 而 KPU 跑 face_detection_320 正常应到 25~30 —— 说明 70ms/帧里有一大半
# 不是花在推理上，是花在 Python 层的重复劳动。
#
#  ① ★核心★ postprocess 每帧重建候选调用表：_cands() 里每次都执行
#     A.astype(np.float32) 和 list(A)。prior_data_320.bin 有几千个 anchor，
#     每帧把 3000+ 个 float32 转成 Python 对象再扔掉 —— 纯白烧 10~20ms，
#     帧率被它砍掉将近一半。v53 把它改成**只建一次**，lambda 收 results
#     做参数，之后直接调用。
#  ② 加耗时探针：诊断行末尾追加 T gf=/run=/post=/dr=（单位 ms，每帧均值）。
#     串口是死的，这是唯一能看到"时间到底花在哪一步"的通道：
#       gf 大 → 取帧/ISP 慢，要降 RGB888P_SIZE
#       run 大而 post 小 → KPU 推理慢
#       post 大 → 后处理还是重
#       dr 大 → LCD 显示拖后腿，加大 DISPLAY_EVERY
#  ③ 尺寸闸放宽：MAX_FACE_W 520->760 / MAX_FACE_H 420->660 / H_LIMIT 400->700。
#     原来人一凑近、脸占画面 2/3 时框就被当"bogus box"丢掉，
#     用户感受就是"我就在它面前，它反而不认我"。
#  ④ STABLE_FRAMES_FIRST 4->3：首次锁定少等一帧（≈70ms）。
#  ⑤ DISPLAY_EVERY 2->3：显示再降一档，省下的直接变成跟踪帧率。
#
# 探针依赖 time.ticks_us()；个别 CanMV 固件没有这个函数，故做降级封装。
try:
    _now_us = time.ticks_us          # [v53] 耗时探针
except AttributeError:
    def _now_us():                   # 固件没有 ticks_us 就退到 ms*1000
        return time.ticks_ms() * 1000

PROF = {"post": 0}                   # [v53] postprocess 累计微秒（诊断行上报）
# ================= v52 改动 =================
# 用户反馈"重新上电后能亮屏，但人脸跟踪很慢"，实测主控整整 45 秒
# 收不到 K230 任何一个 UDP 包（rx 停在 521 不动），说明现场不是"慢"，
# 而是**脚本已经停摆**。根因有三条，全部在 v52 里堵掉：
#
#  ① err_frame>=5 / err_run>=5 时直接 break 出主循环，main() 返回，
#     板子停在 REPL —— 屏幕冻结在最后一帧。用户看到的"黑屏/死机"
#     有一大半是这条。现在改成 machine.reset() 自愈。
#  ② pl.show_image() 没有任何异常保护，一抛异常就冒泡到外层
#     except BaseException，脚本退出、画面定格。现在包 try。
#  ③ WiFi 掉线后 UDP sendto 只是静默失败（不抛错），看着在跑其实
#     一个包都发不出去。现在每 5 秒查一次 WLAN，断了就重连。
#
# 另外为了"跟手"再快一档：
#  · SMOOTH_ALPHA 0.82->0.90（EMA 滞后从约 1.2 帧降到 0.5 帧）
#  · MAX_JUMP 240->400（快速横移不再被限速拖住）
#  · SEND_HZ 20->25（主控 20Hz 轮询，留出余量，不卡在边界上）
#  · DISPLAY_EVERY=2：LCD 每 2 帧刷一次。显示与检测/发包解耦后，
#    帧时间省下 10~20ms，直接换成更高的检测帧率。
#
# 串口在这块板子上是死的（实测 COM33 既无 stdout 也无 REPL 回显），
# 所以 v52 把 fps / 实际发包数塞进已有的每秒诊断行（#s=...），
# 主控的 V1036 日志就能直接看到 K230 内部状态。
HOT_RELOAD = True               # [v54] 打开：改服务端 _version.txt 即自动重启拉新
PC_HOST = ""                    # ★ v46：由 wifi.txt 第3行（PC_IP）填；空=跳过 OTA 检查
PC_PORT = 8123
# [v54] 热重载专用：直连 OTA 服务器，不再依赖 wifi.txt 里的 PC_IP
OTA_HOST = "your.server.example"
OTA_PORT = 18123
OTA_BASE = "/k230-8f3c9a1e"
SCRIPT_NAME = "xiaozhi_face_track.py"
SAVE_PATH = "/sdcard/xz_face.py"
RELOAD_CHECK_MS = 60000      # [v54] 20s->60s 且超时收到 2s：别让检查本身拖帧率
# ★ v46：现场换网支持——/sdcard/wifi.txt 三行文本（UTF-8/ASCII）：
#   第1行 SSID
#   第2行 密码
#   第3行 PC 的 IP（可选，用于镜像调试/OTA 检查；没有就留空或删行）
# 换网只需改这个文件（读卡器/REPL 都行），不用重新部署脚本。
# 文件不存在或行数不够 → 用上面的编译默认值。
WIFI_CFG_PATH = "/sdcard/wifi.txt"
def load_wifi_cfg():
    try:
        f = open(WIFI_CFG_PATH, "r")
        lines = [l.strip() for l in f.read().split("\n")]
        f.close()
    except Exception:
        return None
    vals = [l for l in lines if l and not l.startswith("#")]
    if not vals:
        return None
    cfg = {"ssid": vals[0]}
    if len(vals) >= 2:
        cfg["pass"] = vals[1]
    if len(vals) >= 3:
        cfg["pc"] = vals[2]
    return cfg
_CFG = load_wifi_cfg()
if _CFG:
    WIFI_SSID = _CFG.get("ssid", WIFI_SSID)
    WIFI_PASS = _CFG.get("pass", WIFI_PASS)
    if _CFG.get("pc"):
        MIRROR_IP = _CFG["pc"]       # 镜像抄送到 PC（单播，比广播稳）
        LOG_IP = _CFG["pc"]          # 串口日志同步投递到 PC
        PC_HOST = _CFG["pc"]         # OTA/热重载检查
def log(*a):
    if not DEBUG:
        return
    parts = []
    for x in a:
        try:
            parts.append(str(x))
        except Exception:
            parts.append("?")
    line = "[k230] " + " ".join(parts)
    print(line)
    try:
        if LOG_IP and LOG_SOCK[0] is None:
            import socket as _s
            sk = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
            sk.setblocking(False)
            LOG_SOCK[0] = sk
        LOG_SOCK[0].sendto(line.encode(), (LOG_IP, LOG_PORT))
    except Exception:
        pass
# ---------------- v52 自愈工具（C 层卡死 / 掉网都靠这两个） ----------------
_HW_WDT = [None]

def hw_wdt_start():
    """尽力开启硬件看门狗。K230 各版 CanMV 的 WDT 签名不一致，逐个试。"""
    if not USE_HW_WDT:
        log("hw wdt disabled by config")
        return
    try:
        import machine
    except Exception as e:
        log("hw wdt: no machine module:", e)
        return
    for args in ((0, WD_TIMEOUT_MS), (0,), ()):
        try:
            w = machine.WDT(*args)
            try:
                w.feed()
            except Exception:
                pass
            _HW_WDT[0] = w
            log("hw wdt armed, args=", args)
            return
        except Exception as e:
            last = e
    log("hw wdt unavailable:", last)

def hw_wdt_feed():
    """每帧喂狗。喂失败说明这一路不可用，立刻主动复位（别等硬件到点）。"""
    w = _HW_WDT[0]
    if w is None:
        return
    try:
        w.feed()
    except Exception as e:
        _HW_WDT[0] = None
        log("hw wdt feed fail -> reset:", e)
        try:
            import machine
            machine.reset()
        except Exception:
            pass

def net_keepalive():
    """WiFi 掉了就重连。UDP sendto 在断网时是静默失败，不查这一下会一直以为在发包。"""
    try:
        import network
        sta = network.WLAN(network.STA_IF)
        if not sta.isconnected():
            log("wifi lost -> reconnect", WIFI_SSID)
            try:
                sta.active(True)
                sta.connect(WIFI_SSID, WIFI_PASS)
            except Exception as e:
                log("wifi reconnect fail:", e)
    except Exception as e:
        log("net keepalive err:", e)

def hard_reset(tag):
    """脚本走到无法继续的地方时唯一的正确动作：复位重来，而不是停在 REPL 冻结画面。"""
    log("FATAL", tag, "-> machine.reset()")
    try:
        import machine
        machine.reset()
    except Exception as e:
        log("reset fail:", e)
    return None

def align_up(x, align):
    """
    官方例程里是 from libs.Utils import ALIGN_UP，但这个固件版本(v1.4.0)的
    libs.Utils 里没有它，直接自己实现（就是向上取整到 align 的整数倍）。
    """
    return ((x + align - 1) // align) * align
def http_get(host, port, path, timeout_s=8):
    """极简 HTTP GET，只取 body（板子上的 urequests 不一定有）"""
    import socket
    s = socket.socket()
    try:
        s.settimeout(timeout_s)
        s.connect((host, port))
        s.send(b"GET " + path.encode() + b" HTTP/1.0\r\nHost: a\r\n\r\n")
        d = b""
        while True:
            b = s.recv(2048)
            if not b:
                break
            d += b
    finally:
        try:
            s.close()
        except Exception:
            pass
    i = d.find(b"\r\n\r\n")
    return d[i + 4:] if i >= 0 else d
def connect_wifi(timeout_s=25):
    import network
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    if sta.isconnected():
        log("wifi already up:", sta.ifconfig()[0])
        return sta
    log("connecting wifi:", WIFI_SSID)
    sta.connect(WIFI_SSID, WIFI_PASS)
    t0 = time.ticks_ms()
    while not sta.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_s * 1000:
            raise RuntimeError("wifi connect failed (password? 5G-only AP?)")
        time.sleep_ms(300)
    log("wifi OK, ip =", sta.ifconfig()[0])
    # ★ v46.1：按自己的 IP/掩码算网段定向广播（如 10.x.x.255）。
    #   实测部分路由器不转发 255.255.255.255 受限广播（<ssid> 网就不转），
    #   而网段定向广播在 INTELNET 上验证过能通。算不出来就退回受限广播。
    global BROADCAST_IP
    try:
        _ip, _mask, _gw, _dns = sta.ifconfig()
        _b = [int(x) | (255 ^ int(m))
              for x, m in zip(_ip.split("."), _mask.split("."))]
        BROADCAST_IP = ".".join(str(v) for v in _b)
        log("subnet broadcast:", BROADCAST_IP, "gw:", _gw)
    except Exception as e:
        log("broadcast calc fail:", e, "- keep", BROADCAST_IP)
    return sta
def open_udp():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if USE_BROADCAST:
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass
        dest = (BROADCAST_IP, VOX3_PORT)
    else:
        dest = (VOX3_IP, VOX3_PORT)
    log("udp target =", dest)
    return s, dest
try:
    from media.sensor import CAM_CHN_ID_1
except Exception:
    CAM_CHN_ID_1 = 1            # 拿不到就按官方定义的常量值兜底
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WS = {"srv": None, "cli": None, "rx": b"", "streaming": False, "sent": 0,
      "hs": False, "t0": 0}
JPEG_METHOD = [None]
def ws_drop():
    try:
        if WS["cli"]:
            WS["cli"].close()
    except Exception:
        pass
    WS["cli"] = None
    WS["rx"] = b""
    WS["streaming"] = False
def ws_send_raw(buf):
    """
    所有发往客户端的数据都走这里。
    ★ accept 出来的 socket 继承了监听 socket 的非阻塞属性，直接 send() 很可能
      只发出一部分甚至一个字节都没发（不检查返回值就完全看不出来）。
      实测握手响应用非阻塞 send 根本没出去：PC 等 101 超时关连接，
      我们 500ms 后推 24KB 帧就被 RST，报 ECONNRESET。所以发送期间临时阻塞。
    ★ 全程非阻塞：切阻塞会在慢的时候把整个线程卡住（lwIP 就没人喂了），
      改成 2KB 一片喂、碰到 EAGAIN 歇 2ms 重试，主循环始终在转。
    """
    ok = True
    total = 0
    stall = 0
    while total < len(buf):
        try:
            k = WS["cli"].send(buf[total:total + 2048])
        except Exception as e:
            msg = str(e)
            if "11" in msg or "EAGAIN" in msg or "EWOULDBLOCK" in msg:
                stall += 1
                if stall > 1500:           # 约 3 秒还喂不进去，放弃这条连接
                    log("ws send stalled:", e)
                    ok = False
                    break
                os.exitpoint()
                time.sleep_ms(2)
                continue
            log("ws send fail:", e)
            ok = False
            break
        if not k:
            log("ws send returned", k)
            ok = False
            break
        total += k
        stall = 0
        if stall == 0 and total < len(buf):
            os.exitpoint()          # 大帧分很多片，边发边给网络栈让路
    if not ok:
        log("ws sent only", total, "of", len(buf))
        ws_drop()
    return ok
def ws_send_bin(payload):
    buf = bytes(payload)
    n = len(buf)
    if n < 126:
        head = bytes([0x82, n])
    elif n < 65536:
        head = bytes([0x82, 126]) + n.to_bytes(2, "big")
    else:
        head = bytes([0x82, 127]) + n.to_bytes(8, "big")
    return ws_send_raw(head + buf)
def ws_handshake():
    rx = WS["rx"]
    i = rx.find(b"\r\n\r\n")
    req = rx[: i + 4].decode("utf-8", "ignore")
    WS["rx"] = rx[i + 4:]
    accept = ""
    try:
        key = ""
        for line in req.split("\r\n"):
            if line.lower().startswith("sec-websocket-key:"):
                key = line.split(":", 1)[1].strip()
        if key:
            import ubinascii
            import uhashlib
            h = uhashlib.sha1((key + WS_GUID).encode())
            accept = ubinascii.b2a_base64(h.digest()).decode().strip()
    except Exception as e:
        log("accept calc skip:", e)
    resp = ("HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " + accept + "\r\n\r\n")
    log("sending 101,", len(resp), "bytes")
    if not ws_send_raw(resp.encode()):
        log("101 send FAILED")
        return
    try:
        WS["cli"].setblocking(False)
    except Exception:
        pass
    WS["streaming"] = True
    WS["sent"] = 0
    log("ws handshake done, start pushing")
def http_reply_jpeg():
    """
    普通 HTTP GET 分流（v44.2 新增）：
    请求头无 sec-websocket-key 且无 upgrade: websocket 的，当作普通 HTTP GET，
    返回 HTTP/1.0 200 OK + JPEG body。
    这样墨水屏的 FetchShotJpeg（普通 GET http://<k230>:8080/shot.jpg）就能拿到
    真正的 JPEG 而不是 WS 握手的 101。
    """
    jpg = SHOT_JPEG[0]
    if jpg is None:
        log("http jpeg: no cached jpeg")
        body = b"NO JPEG\n"
    else:
        body = jpg
    hdr = (b"HTTP/1.0 200 OK\r\n"
           b"Content-Type: image/jpeg\r\n"
           b"Content-Length: " + str(len(body)).encode() + b"\r\n"
           b"Connection: close\r\n\r\n")
    log("http jpeg reply:", len(body), "bytes")
    try:
        WS["cli"].send(hdr + body)
    except Exception as e:
        log("http jpeg send fail:", e)
    ws_drop()

def ws_pump_rx():
    try:
        data = WS["cli"].recv(1024)
    except OSError:
        data = None                 # 非阻塞无数据（或超时），不是断线
    except Exception:
        ws_drop()
        return
    if data is None:
        pass                        # 没有新数据也要继续解析已缓冲的
    elif len(data) == 0:
        ws_drop()                   # 对端关了
        return
    else:
        WS["rx"] += data
    if not WS["hs"]:
        if b"\r\n\r\n" in WS["rx"]:
            # 分流：无 WS 头 → 普通 HTTP GET（v44.2 新增）
            req_head = WS["rx"][:WS["rx"].find(b"\r\n\r\n")].lower()
            if b"sec-websocket-key" not in req_head and b"upgrade: websocket" not in req_head:
                log("http request (non-ws) -> reply jpeg")
                http_reply_jpeg()
                return
            ws_handshake()
            WS["hs"] = True
        else:
            return
    while len(WS["rx"]) >= 2:
        b1 = WS["rx"][1]
        opcode = WS["rx"][0] & 0x0F
        masked = (b1 & 0x80) != 0
        n = b1 & 0x7F
        hdr = 2
        if n == 126:
            if len(WS["rx"]) < 4:
                return
            n = (WS["rx"][2] << 8) | WS["rx"][3]
            hdr = 4
        elif n == 127:
            if len(WS["rx"]) < 10:
                return
            n = 0
            for k in range(2, 10):
                n = (n << 8) | WS["rx"][k]
            hdr = 10
        mask = b""
        if masked:
            if len(WS["rx"]) < hdr + 4:
                return
            mask = WS["rx"][hdr: hdr + 4]
            hdr += 4
        if len(WS["rx"]) < hdr + n:
            return
        payload = bytearray(WS["rx"][hdr: hdr + n])
        WS["rx"] = WS["rx"][hdr + n:]
        if masked:
            for k in range(n):
                payload[k] ^= mask[k & 3]
        if opcode == 0x8:
            ws_drop()
            return
        if opcode == 0x1:
            try:
                txt = payload.decode("utf-8", "ignore")
                if '"set_camera_enabled"' in txt:
                    if "true" in txt:
                        WS["streaming"] = True
                        WS["sent"] = 0
                        log("ws stream ON")
                    else:
                        WS["streaming"] = False
                        log("ws stream OFF")
            except Exception:
                pass
def _accept_one(srv):
    """
    摄像头出流时，直接 srv.accept() 一次都拿不到连接，但 PC 的 connect()
    明明是成功的（SYN 被 lwIP 处理、SYN-ACK 也回了）—— 连接躺在 backlog 里
    没人去掏。这里先让 socket 层把监听口 poll 一遍（uselect 会触发一次
    lwIP 的 event 检查），再 accept。拿不到就返回 (None, None)。
    """
    try:
        import uselect
        r = uselect.select([srv], [], [], 0)
        if not r or not r[0]:
            return (None, None)
    except Exception:
        pass                    # 没有 uselect 就直接裸 accept
    try:
        return srv.accept()
    except Exception:
        return (None, None)
def ws_init(port):
    import socket
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    srv.setblocking(False)
    WS["srv"] = srv
    log("camera_ws listening :" + str(port))
def ws_poll():
    if WS["srv"] is None:
        return
    got = _accept_one(WS["srv"])
    if got is not None and got[0] is not None:
        c, addr = got
        if WS["cli"] is not None:
            log("replace stale client")
            try:
                WS["cli"].close()
            except Exception:
                pass
        WS["cli"] = c
        try:
            c.setblocking(False)
        except Exception:
            pass
        WS["rx"] = b""
        WS["hs"] = False
        WS["streaming"] = False
        WS["t0"] = time.ticks_ms()
        log("ws client:", addr)
    if WS["cli"] is None:
        return
    if WS["t0"] and time.ticks_diff(time.ticks_ms(), WS["t0"]) > 20000:
        log("ws idle timeout -> drop")
        ws_drop()
        return
    ws_pump_rx()
def ws_push(jpeg, w, h):
    """推一帧。返回 True=真的发出去了（调用方靠它决定要不要换下一张图）。"""
    if WS["cli"] is None or not WS["streaming"]:
        return False
    if len(jpeg) < 512:         # ParseCamFrame 有下限，太小的帧主控会丢
        log("jpeg too small:", len(jpeg))
        return False
    hdr = bytearray(16)
    hdr[0] = 0x43               # 'C'
    hdr[2] = 0x01               # MJPG
    jl = len(jpeg).to_bytes(4, "little")
    hdr[4], hdr[5], hdr[6], hdr[7] = jl[0], jl[1], jl[2], jl[3]
    wb = w.to_bytes(2, "little")
    hb = h.to_bytes(2, "little")
    hdr[12], hdr[13] = wb[0], wb[1]
    hdr[14], hdr[15] = hb[0], hb[1]
    if not ws_send_bin(bytes(hdr) + jpeg):
        WS["err"] = WS.get("err", 0) + 1
        return False            # send 已经失败了，别再打印 pushed 骗自己
    WS["sent"] += 1
    log("pushed frame", WS["sent"], len(jpeg), "%dx%d" % (w, h))
    if WS["sent"] >= SHOT_FRAMES:
        log("shot done, close client")
        ws_drop()
    return True
def _encode_at(img, q):
    """用指定质量试三种出图方式，返回 JPEG bytes 或 None"""
    for name in ("compressed", "to_jpeg"):
        try:
            d = bytes(getattr(img, name)(quality=q).bytearray())
            if len(d) > 512 and d[0] == 0xFF and d[1] == 0xD8:
                log("  jpeg", name, "q" + str(q), "->", len(d))
                return d
        except Exception as e:
            log("  jpeg", name, "fail:", e)
    try:
        img.save("/sdcard/_shot.jpg", quality=q)
        f = open("/sdcard/_shot.jpg", "rb")
        d = f.read()
        f.close()
        if len(d) > 512 and d[0] == 0xFF and d[1] == 0xD8:
            log("  jpeg save q" + str(q), "->", len(d))
            return d
    except Exception as e:
        log("  jpeg save fail:", e)
    return None
def probe_jpeg(img):
    """
    开机试一次，确认这条路能出 JPEG。
    ★ 传进来的必须是 image.Image：pl.get_frame() 给的是 ulab ndarray（RGBP888），
      它没有任何图像方法（实测 compressed/to_jpeg/save 全无），出不了 JPEG。
    """
    log("--- probing jpeg ---")
    try:
        log("img ALL methods:", [m for m in dir(img) if not m.startswith("_")])
    except Exception as e:
        log("dir fail:", e)
    j = _encode_at(img, JPEG_QUALITY)
    JPEG_METHOD[0] = "ok" if j else None
    log("jpeg probe =", JPEG_METHOD[0])
def encode_jpeg(img):
    """
    从高质量往下试，直到压到 JPEG_TARGET_BYTES 以下。
    ★ 为什么要限大小：实测 640x480 q60 出来 197KB，lwIP 一帧 send 直接 EAGAIN，
      一个字节都发不出去（主控那边也扛不住这么大的帧）。
    """
    best = None
    for q in (JPEG_QUALITY, 40, 25, 15):
        d = _encode_at(img, q)
        if not d:
            continue
        if best is None or len(d) < len(best):
            best = d
        if len(d) <= JPEG_TARGET_BYTES:
            return d
    return best
def grab_jpeg(pl):
    """
    取一帧 JPEG。
    ★ 优先级：先 snapshot(CHN_ID_1) —— 它出来的是真实摄像头帧，JPEG 内容正确；
      失败再用 pl.osd_img —— 它虽然不会触发双通道卡死，但格式是 ARGB8888，
      JPEG 编码器容易把它压成花屏。
    ★ 安全前提：do_shot() 被调用时 tracking 主循环已经卡住（执行 do_shot），所以
      snapshot() 不会跟 get_frame() 交替运行，不会触发"双通道混用卡死"。
    """
    try:
        j = encode_jpeg(pl.sensor.snapshot(chn=CAM_CHN_ID_1))
        if j:
            log("jpeg <- snapshot chn1", len(j))
            return j
        log("snapshot chn1 -> no jpeg")
    except Exception as e:
        log("snapshot chn1 fail:", e)
    try:
        j = encode_jpeg(pl.osd_img)
        if j:
            log("jpeg <- osd_img", len(j))
            return j
    except Exception as e:
        log("osd_img fail:", e)
    return None
def grab_shot_pair(pl):
    """
    一次抓帧，出两张 JPEG：(小图, 大图)。
      小图 240x240 —— 给小智那块 240x240 屏显示用。主控内部 SRAM 只剩 24~33KB，
                      自己解 640x480 要 600KB RGB565，不现实；这边顺手缩好最省。
      大图 640x480 —— 给识图服务器用（要看得清细节）。
    ★ 只 snapshot 一次：snapshot() 和跟踪用的 get_frame() 交替是把板子搞死的
      老坑，能少一次是一次。
    ★ 小图出不来不影响识图（返回 None，屏幕就不显示照片而已）。
    """
    img = None
    try:
        img = pl.sensor.snapshot(chn=CAM_CHN_ID_1)
    except Exception as e:
        log("snapshot chn1 fail:", e)
    if img is None:
        try:
            img = pl.osd_img          # 兜底：ARGB8888，JPEG 容易花，但聊胜于无
        except Exception:
            return (None, None)
    big = encode_jpeg(img)
    if big:
        log("jpeg big", len(big))
    return (None, big)
def shot_server_init():
    """拍照指令的 UDP 口。非阻塞，出流时也能收。"""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", SHOT_CMD_PORT))
    except Exception as e:
        log("shot udp bind fail:", e)
        return None
    s.setblocking(False)
    log("shot cmd udp :" + str(SHOT_CMD_PORT))
    return s
def maybe_shot(sock):
    """
    出流时也能收的指令口。返回指令字符串（"" 表示没指令）。
      SHOT    —— 拍一张，停流出后起 :8080 等服务小智来取
      RELOAD  —— 干净退出并复位，让自启重新拉一份代码（= 免拔电的 OTA）
      BOOTFIX —— 把 /sdcard/main.py 换成会打错误日志的那版
      EXIT    —— 干净退出，不复位（留着 REPL 好手动灌代码）
    """
    if sock is None:
        return ""
    try:
        d, addr = sock.recvfrom(64)
    except OSError:
        return ""
    except Exception:
        return ""
    try:
        txt = d.decode("utf-8", "ignore").strip().upper()
    except Exception:
        return ""
    for kw in ("SHOT", "RELOAD", "BOOTFIX", "EXIT"):
        if kw in txt:
            log("cmd", txt, "from", addr)
            return kw
    return ""
BOOT_MAIN = r'''SSID = "your-ssid"
PASS = "REDACTED-WIFI-PASS"
SERVERS = [
    ("your.server.example", 18123),
]
# ★ v46：现场换网——/sdcard/wifi.txt 三行：SSID / 密码 / PC_IP(可选)
#   改这个文件即可换网；PC_IP 给了就把它加进 OTA 兜底服务器列表
try:
    _f = open("/sdcard/wifi.txt", "r")
    _ls = [l.strip() for l in _f.read().split("\n")]
    _f.close()
    _ls = [l for l in _ls if l and not l.startswith("#")]
    if _ls:
        SSID = _ls[0]
        if len(_ls) >= 2:
            PASS = _ls[1]
        if len(_ls) >= 3:
            SERVERS = SERVERS + [(_ls[2], 8123)]
except Exception:
    pass
NAME = "k230-8f3c9a1e/xiaozhi_face_track.py"
CACHE = "/sdcard/xz_face.py"   # 最后兜底：网上都拉不到就用它
SKIP_FILE = "/sdcard/xz_skip"
WAIT_S = 8
def log(*a):
    print("[boot]", *a)
def httpget(host, port, path, timeout_s=6):
    import socket
    s = socket.socket()
    d = b""
    try:
        s.settimeout(timeout_s)
        s.connect((host, port))
        s.send(b"GET " + path.encode() + b" HTTP/1.0\r\nHost: a\r\n\r\n")
        while True:
            b = s.recv(2048)
            if not b:
                break
            d += b
    finally:
        try:
            s.close()
        except Exception:
            pass
    i = d.find(b"\r\n\r\n")
    return d[i + 4:] if i >= 0 else d
def main():
    import os
    import time
    try:
        if SKIP_FILE in os.listdir("/sdcard"):
            log("xz_skip found, do nothing")
            return
    except Exception:
        pass
    log("autorun in %ds ... Ctrl+C to cancel" % WAIT_S)
    try:
        for _ in range(WAIT_S * 2):
            time.sleep_ms(500)
    except KeyboardInterrupt:
        log("cancelled by user, REPL is yours")
        return
    import network
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    if not sta.isconnected():
        log("connecting wifi", SSID)
        sta.connect(SSID, PASS)
        for _ in range(100):
            if sta.isconnected():
                break
            time.sleep_ms(300)
    log("ip =", sta.ifconfig()[0])
    body = b""
    for host, port in SERVERS:
        try:
            b = httpget(host, port, "/" + NAME, timeout_s=6)
            log("try %s:%d ->" % (host, port), len(b), "bytes")
            if len(b) >= 500:
                body = b
                break
        except Exception as e:
            log("http fail %s:%d:" % (host, port), e)
    src = None
    if len(body) >= 500:
        src = body.decode("utf-8")
        try:
            f = open(CACHE, "wb")
            f.write(body)
            f.close()
            log("cache updated")
        except Exception as e:
            log("cache write fail (readonly?):", e)
    else:
        log("no net version, fall back to SD cache")
        try:
            f = open(CACHE, "rb")
            src = f.read().decode("utf-8")
            f.close()
            log("using SD cache", len(src), "bytes")
        except Exception as e:
            log("no cache either:", e)
    if src is None or len(src) < 500:
        raise RuntimeError("no usable script (net=%d bytes)" % len(body))
    exec(src)
try:
    main()
except BaseException as e:
    try:
        import sys
        sys.print_exception(e)
    except Exception:
        print("EXC", e)
    print("[boot] stopped, REPL is free")
'''
def install_main():
    """
    把 /sdcard/main.py 换成会打错误日志的那版。
    ★ 已有文件不能直接 open("w")（EPERM）也不能 rename（EBUSY，被占用），
      只能 remove() 之后重新 open("wb") 建一个 —— 这条是实测出来的。
    """
    try:
        try:
            os.remove("/sdcard/main.py")
        except Exception as e:
            log("main remove:", e)
        f = open("/sdcard/main.py", "wb")
        f.write(BOOT_MAIN.encode())
        f.close()
        f = open("/sdcard/main.py", "rb")
        d = f.read()
        f.close()
        log("main.py installed,", len(d), "bytes")
        return True
    except Exception as e:
        log("main install fail:", e)
        return False
def do_shot(pl, face_det, sock, dest):
    """
    UDP 收到 SHOT 后的完整流程。
    ★★ 两种模式：
      SHOT_INLINE=True（v30 新默认）—— 只抓一帧缓存起来，**不停流出、不复位**。
        主循环照常跑跟踪，ws_poll() 每帧都在等小智来连；连上就把缓存帧推出去。
        好处：屏幕不灰屏、跟踪几乎不中断（用户最烦的"拍完照黑半天"消失了）。
      SHOT_INLINE=False（v29 老路）—— 抓帧 → destroy 停流出 → 起 WS 服务 → reset。
    ★ 老路的顺序不能变：出流时 lwIP 连 accept 都跑不动，必须 destroy() 之后再
      listen/accept；而 destroy() 之后 create() 会把板子卡死，所以只能 reset
      （/sdcard/main.py 的自启会把跟踪拉回来）。
    ★ 老路停流出 + 重启这十几秒主控收不到坐标会去扫描找人（舵机乱转），所以全程发
      "人在正中"的心跳，把云台按住。
    """
    small = None
    jpg = None
    try:
        small, jpg = grab_shot_pair(pl)
        log("shot jpeg", len(jpg) if jpg else 0, "small", len(small) if small else 0)
    except Exception as e:
        log("grab fail:", e)
    if jpg:
        SHOT_JPEG[0] = jpg
        SHOT_JPEG_AT[0] = time.ticks_ms()
    if small:
        SHOT_JPEG_SMALL[0] = small
    SHOT_PUSHED[0] = 0        # 重新计数：第 1 帧推小图，之后推大图
    if SHOT_INLINE:
        log("shot cached, keep tracking (inline)")
        return
    try:
        if face_det is not None:
            face_det.deinit()
    except Exception:
        pass
    try:
        pl.destroy()
        log("camera stopped -> tcp alive now")
    except Exception as e:
        log("destroy fail:", e)
    pl = None
    face_det = None
    if jpg and ENABLE_CAM_WS:
        try:
            if WS["srv"] is None:
                ws_init(CAM_WS_PORT)
        except Exception as e:
            log("ws init fail:", e)
        WS["sent"] = 0
        WS["streaming"] = False
        try:
            t_flush = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t_flush) < 300:
                try:
                    c, a = WS["srv"].accept()
                    c.close()
                    log("flush stale", a)
                except OSError:
                    break
                time.sleep_ms(5)
        except Exception as e:
            log("flush err:", e)
        hb_t = 0
        beat_t = 0
        t_end = time.ticks_ms() + SHOT_SERVE_MS
        got = False
        while time.ticks_diff(t_end, time.ticks_ms()) > 0:
            now = time.ticks_ms()
            os.exitpoint()
            if time.ticks_diff(now, hb_t) >= 100:
                hb_t = now
                try:
                    sock.sendto(HB_LINE, dest)
                except Exception:
                    pass
            if time.ticks_diff(now, beat_t) >= 2000:
                beat_t = now
                log("serving ... cli=", WS["cli"] is not None,
                    "sent=", WS["sent"])
            ws_poll()
            if WS["streaming"]:
                ws_push(jpg, RGB888P_SIZE[0], RGB888P_SIZE[1])
            if WS["sent"] >= SHOT_FRAMES:
                log("frames delivered", WS["sent"])
                got = True
                break
            time.sleep_ms(5)
        if not got:
            log("shot serve timeout, sent =", WS["sent"])
        ws_drop()
        try:
            if WS["srv"]:
                WS["srv"].close()
        except Exception:
            pass
        WS["srv"] = None
    log("shot done -> reset")
    try:
        import machine
        machine.reset()
    except Exception as e:
        log("reset fail:", e)
def build_face_app():
    from libs.PipeLine import ScopedTiming
    from libs.AIBase import AIBase
    from libs.AI2D import Ai2d
    import nncase_runtime as nn
    import ulab.numpy as np
    import aidemo
    class FaceDetectionApp(AIBase):
        def __init__(self, kmodel_path, model_input_size, anchors,
                     confidence_threshold=0.5, nms_threshold=0.2,
                     rgb888p_size=[640, 480], display_size=[640, 480], debug_mode=0):
            super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
            self.kmodel_path = kmodel_path
            self.model_input_size = model_input_size
            self.confidence_threshold = confidence_threshold
            self.nms_threshold = nms_threshold
            self.anchors = anchors
            self.rgb888p_size = [align_up(rgb888p_size[0], 16), rgb888p_size[1]]
            self.display_size = [align_up(display_size[0], 16), display_size[1]]
            self.debug_mode = debug_mode
            self.form = None            # 记住 aidemo 哪种入参形式能work
            self._cand_tab = None       # [v53] 候选调用表缓存（只建一次）
            self.ai2d = Ai2d(debug_mode)
            self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
                                     np.uint8, np.uint8)
        def config_preprocess(self, input_image_size=None):
            with ScopedTiming("set preprocess config", self.debug_mode > 0):
                ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
                top, bottom, left, right = self.get_padding_param()
                self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])
                self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                                [1, 3, self.model_input_size[1], self.model_input_size[0]])
        def _build_cands(self):
            """
            [v53] ★只调用一次★，结果缓存在 self._cand_tab。

            v52 及以前每次 postprocess 都会重跑这个函数：里面那句
            A.astype(np.float32) 和 list(A) 要把几千个 anchor 转成 Python
            对象再丢掉，每帧一次 —— 这是 14fps 的最大元凶。
            现在 lambda 收 r(results) 当参数，同一张表可以反复用。
            """
            f = aidemo.face_det_post_process
            c = self.confidence_threshold
            n = self.nms_threshold
            mis = self.model_input_size
            rs = self.rgb888p_size
            A = self.anchors
            mis_t = (int(mis[0]), int(mis[1]))
            rs_t = (int(rs[0]), int(rs[1]))
            out = [
                ("int-size", lambda r: f(c, n, int(mis[0]), A, rs, r)),   # 0 已验证可用（尺寸=int）
                ("model-list", lambda r: f(c, n, mis, A, rs, r)),         # 1 官方例程写法
                ("tuple-size", lambda r: f(c, n, mis_t, A, rs_t, r)),     # 2 尺寸用 tuple
                ("five-arg", lambda r: f(c, n, mis, A, r)),               # 3 五个参数（不带尺寸）
                ("rs-last", lambda r: f(c, n, mis, A, r, rs)),            # 4 results 在尺寸前面
            ]
            try:
                A32 = A.astype(np.float32)
                out.append(("a32", lambda r: f(c, n, mis, A32, rs, r)))   # 5 anchors 转 float32
            except Exception:
                pass
            try:
                Al = list(A)
                out.append(("alist", lambda r: f(c, n, mis, Al, rs, r)))  # 6 anchors 转 list
            except Exception:
                pass
            return out
        def _pick_form(self, results):
            table = self._build_cands()
            for i in range(len(table)):
                name, fn = table[i]
                try:
                    r = fn(results)
                    log("aidemo form", i, name, "OK, got", len(r) if hasattr(r, "__len__") else r)
                    self._cand_tab = table      # [v53] 定下来，之后不再重建
                    return i
                except Exception as e:
                    log("aidemo form", i, name, "fail:", e)
            self._cand_tab = None
            log("!! 所有入参形式都失败，后面只会返回空")
            return -1
        def postprocess(self, results):
            _p0 = _now_us()
            try:
                with ScopedTiming("postprocess", self.debug_mode > 0):
                    if self.form is None:
                        self.form = self._pick_form(results)
                    if self.form < 0 or self._cand_tab is None:
                        return []
                    try:
                        post_ret = self._cand_tab[self.form][1](results)
                    except Exception as e:
                        log("aidemo call fail:", e)
                        return []
                    if len(post_ret) == 0:
                        return post_ret
                    return post_ret[0]
            finally:
                PROF["post"] += time.ticks_diff(_now_us(), _p0)
        def get_padding_param(self):
            dst_w = self.model_input_size[0]
            dst_h = self.model_input_size[1]
            ratio_w = dst_w / self.rgb888p_size[0]
            ratio_h = dst_h / self.rgb888p_size[1]
            ratio = min(ratio_w, ratio_h)
            new_w = int(ratio * self.rgb888p_size[0])
            new_h = int(ratio * self.rgb888p_size[1])
            dw = (dst_w - new_w) / 2
            dh = (dst_h - new_h) / 2
            return 0, int(round(dh * 2 + 0.1)), 0, int(round(dw * 2 - 0.1))
        def draw_result(self, pl, dets):
            with ScopedTiming("display_draw", self.debug_mode > 0):
                pl.osd_img.clear()
                if dets:
                    for det in dets:
                        x, y, w, h = map(lambda v: int(round(v, 0)), det[:4])
                        x = x * self.display_size[0] // self.rgb888p_size[0]
                        y = y * self.display_size[1] // self.rgb888p_size[1]
                        w = w * self.display_size[0] // self.rgb888p_size[0]
                        h = h * self.display_size[1] // self.rgb888p_size[1]
                        pl.osd_img.draw_rectangle(x, y, w, h,
                                                  color=(255, 255, 0, 255), thickness=2)
    anchors = np.fromfile(ANCHORS_PATH, dtype=np.float)
    app = FaceDetectionApp(KMODEL_PATH, model_input_size=MODEL_INPUT_SIZE, anchors=anchors,
                           confidence_threshold=CONF_TH, nms_threshold=NMS_TH,
                           rgb888p_size=RGB888P_SIZE, display_size=DISPLAY_SIZE, debug_mode=0)
    app.config_preprocess()
    return app
def box_ok(d):
    """★v49 统一闸：尺寸 + 宽高比。不合格的框不参与挑选。

    原来这两道闸在主循环里、挑选之后才判，于是 pick_* 可能挑中一个
    34x47 的小误检框，再被丢掉 —— 真脸那一帧就被白白浪费了。
    """
    if len(d) < 4:
        return False
    bw, bh = d[2], d[3]
    if bw > MAX_FACE_W or bh > MAX_FACE_H or bw < MIN_FACE_W or bh < MIN_FACE_H:
        return False
    if bh > MAX_FACE_H_LIMIT or bh <= 0:
        return False
    ar = bw * 1.0 / bh
    if ar < MIN_ASPECT or ar > MAX_ASPECT:
        return False
    return True
def pick_biggest(dets):
    best = None
    best_area = -1
    for d in dets:
        if not box_ok(d):
            continue
        area = d[2] * d[3]
        if area > best_area:
            best_area = area
            best = d
    return best
def pick_stable(dets, prev_cx, prev_cy):
    """
    ★★ 选"离上一次最近"的那张脸，而不是最大的那张。
    为什么必须这样：画面里只要出现第二个候选（另一个人、海报/屏幕上的脸、
    或者模型偶尔冒出来的误检框），"取最大"就会在它们之间来回切换 ——
    实测坐标 8 秒里 cx 从 171 跳到 557，云台被带着左右猛甩，
    用户看到的就是"舵机一直在抖"。
    改成按位置就近锁定后，一旦盯上某个目标，除非它消失，否则不会跳走。
    prev_cx / prev_cy 是**原始像素**坐标（镜像/归一化之前），None 表示还没锁定。
    """
    if not dets:
        return None
    if prev_cx is None or prev_cy is None:
        return pick_biggest(dets)
    best = None
    best_d = None
    for d in dets:
        if not box_ok(d):   # ★v49 只在小误检框/真脸里挑合格的那些
            continue
        cx = d[0] + d[2] / 2.0
        cy = d[1] + d[3] / 2.0
        dist = (cx - prev_cx) ** 2 + (cy - prev_cy) ** 2
        if best_d is None or dist < best_d:
            best_d = dist
            best = d
    return best
def main():
    log("=== xiaozhi_face_track", SCRIPT_VER, "===")
    log("wifi cfg:", WIFI_SSID, "| src:", "wifi.txt" if _CFG else "builtin",
        "| pc:", PC_HOST or "-")
    connect_wifi()
    sock, dest = open_udp()
    if ENABLE_CAM_WS:
        try:
            ws_init(CAM_WS_PORT)
        except Exception as e:
            log("ws init fail (拍照可能不可用):", e)
    from libs.PipeLine import PipeLine
    pl = None
    face_det = None
    state = {"ver": None, "next_check": 0, "reload": False}
    try:
        log("init camera ...")
        pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_size=DISPLAY_SIZE,
                      display_mode=DISPLAY_MODE)
        pl.create()
        if HMIRROR:
            try:
                pl.sensor.set_hmirror(True)
            except Exception:
                pass
        if VMIRROR:
            try:
                pl.sensor.set_vflip(True)
            except Exception:
                pass
        shot_sock = None
        try:
            probe_jpeg(pl.osd_img)
        except Exception as e:
            log("jpeg probe fail:", e)
        try:
            shot_sock = shot_server_init()
        except Exception as e:
            log("shot server fail:", e)
        log("loading model ...")
        face_det = build_face_app()
        log("model ready, start tracking")
        hw_wdt_start()      # [v52] 模型就位后再开狗（加载模型本身可能要几秒）
        send_period = 1000 // max(1, SEND_HZ)
        noface_period = 1000 // max(1, NOFACE_HZ)
        last_send = 0
        last_noface = 0
        held = None             # 上一次的人脸坐标 [cx,cy,w,h,conf]
        lock_cx = None          # 已锁定目标的**原始像素**中心（pick_stable 用）
        lock_cy = None
        stable_n = 0            # 连续"位置稳定"的帧数（时间一致性过滤用）
        prev_cxv = None         # 上一帧的中心（像素），用于算帧间位移
        prev_cyv = None
        last_face_ms = 0
        tx_t = [0]              # 上次发送时刻（放列表里，闭包里好改）
        tx_log_t = [0]          # 上次把坐标写进 UDP 日志的时刻（降频用）
        shot_t0 = None          # 拍照模式开始时刻（超时兜底，别饿死跟踪）
        shot_err = 0            # 拍照连续失败次数（>=3 就放弃这条路）
        diag_t = [0]            # ★v47 上次发诊断行的时刻
        DIAG = ["boot"]          # ★v47 当前这一帧的人脸判定结果（发去 VOX3 看日志）
        tx_cnt = [0]            # [v52] 实际发出去的 F 包总数（诊断行上报）
        prof = [0, 0, 0, 0]     # [v53] 累计微秒：0=get_frame 1=run 2=draw 3=帧数
        cur_fps = [0.0]         # [v52] 最近一个 5 秒窗口的帧率（诊断行上报）
        last_ok = time.ticks_ms()   # [v52] 上一次"成功跑完一帧"的时刻（软件看门狗）
        display_n = 0           # [v52] 显示分频计数
        def send_face(v):
            n = time.ticks_ms()
            if time.ticks_diff(n, tx_t[0]) < send_period:
                return
            tx_t[0] = n
            tx_cnt[0] += 1      # [v52] 诊断用
            line = "F,%d,%d,%d,%d,%d\n" % (v[0], v[1], v[2], v[3], v[4])
            try:
                sock.sendto(line.encode(), dest)
                if MIRROR_TO_PC and MIRROR_IP:
                    sock.sendto(line.encode(), (MIRROR_IP, MIRROR_PORT))
            except Exception as e:
                log("sendto fail:", e)
            if DEBUG:
                print(line.strip())
            if LOG_TX:
                n2 = time.ticks_ms()
                if time.ticks_diff(n2, tx_log_t[0]) >= 400:
                    tx_log_t[0] = n2
                    log("TX", line.strip())
        err_run = 0
        err_frame = 0
        t0 = time.ticks_ms()
        fps_t = time.ticks_ms()
        frames = 0
        last_ok = time.ticks_ms()   # [v52] 看门狗基准：进循环这一刻
        while True:
            os.exitpoint()
            hw_wdt_feed()       # [v52] 喂硬件狗
            # [v52] 软件看门狗：一帧都没跑成功超过 6 秒，说明循环还在转但卡在
            #       某一步（多半是取帧/显示阻塞）。主动复位，别等人来拔电。
            if time.ticks_diff(time.ticks_ms(), last_ok) > WD_TIMEOUT_MS:
                log("watchdog: %d ms without a frame -> reset" % WD_TIMEOUT_MS)
                hard_reset("frame stall")
            cmd = maybe_shot(shot_sock)
            if cmd:
                if cmd == "EXIT":
                    log("remote exit -> drop to repl")
                    break
                if cmd == "BOOTFIX":
                    install_main()
                elif cmd == "RELOAD":
                    log("remote reload -> reset")
                    state["reload"] = True
                    break
                else:
                    do_shot(pl, face_det, sock, dest)
                    last_ok = time.ticks_ms()   # [v52] 拍照+等取图可能几十秒
            if ENABLE_CAM_WS:
                ws_poll()
                if WS["streaming"]:
                    if shot_t0 is None:
                        shot_t0 = time.ticks_ms()
                        log("shot mode ON")
                    if time.ticks_diff(time.ticks_ms(), shot_t0) > SHOT_TIMEOUT_MS:
                        log("shot mode timeout -> drop client")
                        ws_drop()
                        shot_t0 = None
                    else:
                        if shot_err >= 3:
                            log("shot failed 3x, give up and drop client")
                            ws_drop()
                            shot_t0 = None
                            shot_err = 0
                            continue
                        if SHOT_PUSHED[0] == 0 and SHOT_JPEG_SMALL[0] is not None:
                            jpg = SHOT_JPEG_SMALL[0]
                            pw, ph = SHOT_SMALL[0], SHOT_SMALL[1]
                        else:
                            jpg = SHOT_JPEG[0]
                            pw, ph = RGB888P_SIZE[0], RGB888P_SIZE[1]
                        if jpg is None or (time.ticks_diff(
                                time.ticks_ms(), SHOT_JPEG_AT[0]) > SHOT_CACHE_MS):
                            try:
                                sm, bg = grab_shot_pair(pl)
                                if bg:
                                    SHOT_JPEG[0] = bg
                                    SHOT_JPEG_AT[0] = time.ticks_ms()
                                if sm:
                                    SHOT_JPEG_SMALL[0] = sm
                            except Exception as e:
                                log("regrab fail:", e)
                            jpg = SHOT_JPEG_SMALL[0] if (
                                SHOT_PUSHED[0] == 0 and SHOT_JPEG_SMALL[0]) else SHOT_JPEG[0]
                            pw, ph = (SHOT_SMALL[0], SHOT_SMALL[1]) if (
                                SHOT_PUSHED[0] == 0 and SHOT_JPEG_SMALL[0]) else (
                                RGB888P_SIZE[0], RGB888P_SIZE[1])
                        if jpg:
                            if ws_push(jpg, pw, ph):
                                SHOT_PUSHED[0] += 1
                        else:
                            shot_err += 1
                            log("shot err", shot_err)
                        continue
                else:
                    shot_t0 = None
            _pg = _now_us()     # [v53] 取帧计时开始
            try:
                img = pl.get_frame()
                err_frame = 0
                prof[0] += time.ticks_diff(_now_us(), _pg)
            except Exception as e:
                err_frame += 1
                log("frame fail(%d): %s" % (err_frame, e))
                if err_frame >= 5:
                    # [v52] 原来这里 break —— 脚本退出、画面冻结在最后一帧，
                    #       用户看到的就是"黑屏/卡死"。改成复位自愈。
                    hard_reset("camera dead")
                    break
                time.sleep_ms(200)
                continue
            _pr = _now_us()     # [v53] 推理计时开始（含 preprocess/postprocess）
            try:
                dets = face_det.run(img)
                err_run = 0
                prof[1] += time.ticks_diff(_now_us(), _pr)
            except Exception as e:
                err_run += 1
                log("run fail(%d): %s" % (err_run, e))
                dets = []
                if err_run >= 5:
                    hard_reset("inference dead")   # [v52] 同上，别停在 REPL
                    break
            now = time.ticks_ms()
            last_ok = now       # [v52] 这一帧跑通了，喂软件狗
            if HOT_RELOAD and time.ticks_diff(now, state["next_check"]) >= 0:
                state["next_check"] = now + RELOAD_CHECK_MS
                try:
                    v = http_get(OTA_HOST, OTA_PORT,
                                 OTA_BASE + "/_version.txt", 2).strip()
                    if v == b"stop":
                        log("remote stop -> exit cleanly")
                        break
                    if state["ver"] is None:
                        state["ver"] = v
                        log("ota version =", v)
                    elif v != state["ver"]:
                        log("new version", v, "-> reset")
                        state["reload"] = True
                        break
                except Exception as e:
                    log("ota check fail:", e)
            jump_skip = False   # ★v51 本帧命中面积跳变 -> 丢掉这一帧，别去追新目标
            try:
                best = pick_stable(dets, lock_cx, lock_cy) if dets else None
                if best is None and not dets:
                    DIAG[0] = "NODET"
            except Exception as e:
                log("pick err:", e)
                DIAG[0] = "PICKERR %s" % e
                best = None
            if best is not None:
                bw = best[2]
                bh = best[3]
                if (bw > MAX_FACE_W or bh > MAX_FACE_H or bw < MIN_FACE_W
                        or bh < MIN_FACE_H or bh > MAX_FACE_H_LIMIT):
                    log("drop bogus box", bw, "x", bh)
                    DIAG[0] = "SZ %dx%d" % (bw, bh)
                    best = None
                if best is not None and bh > 0:
                    ar = bw * 1.0 / bh
                    if ar < MIN_ASPECT or ar > MAX_ASPECT:
                        log("drop bad aspect %.2f" % ar, bw, "x", bh)
                        DIAG[0] = "AR %.2f %dx%d" % (ar, bw, bh)
                        best = None
                if best is not None:
                    x, y, w, h = best[0], best[1], best[2], best[3]
                    # ★ v48 面积跳变守卫：画面里出现"像人脸的物品"时，框会突然
                    #   变大/变小（换目标），这时清掉稳定计数、别马上去追。
                    #   真脸靠近/远离是渐变的，相邻帧面积不会瞬变 2 倍以上。
                    if held is not None and w > 0 and h > 0:
                        _pa = ((held[2] / 1000.0 * RGB888P_SIZE[0])
                               * (held[3] / 1000.0 * RGB888P_SIZE[1]))
                        if _pa > 1.0:
                            _ratio = (w * h) / _pa
                            if _ratio > 2.2 or _ratio < 0.45:
                                stable_n = 0
                                prev_cxv = None
                                prev_cyv = None
                                jump_skip = True
                                DIAG[0] = "JUMP %.1fx" % _ratio
                    lock_cx = x + w / 2.0
                    lock_cy = y + h / 2.0
                    if (prev_cxv is None
                            or abs(lock_cx - prev_cxv) > STABLE_DIST
                            or abs(lock_cy - prev_cyv) > STABLE_DIST):
                        stable_n = 0
                    else:
                        stable_n += 1
                    prev_cxv = lock_cx
                    prev_cyv = lock_cy
                    # ★ v51（本次核心修复）：
                    #   v50 及以前，追踪期也要求"连续 STABLE_FRAMES_TRACK 帧、
                    #   且帧间位移 < STABLE_DIST"才发一个包。人一走动，位移立刻
                    #   超过阈值 → stable_n 反复清零 → 一个包都不发，
                    #   云台就"静静不动"；等站定几帧再猛追。用户说的
                    #   "跟踪太慢 / 有时静一下"就是这条。
                    #   现在：首次锁定仍要 STABLE_FRAMES_FIRST 帧（抗误检），
                    #   一旦锁上（held 有值）就逐帧直发；误检由几何闸、
                    #   面积跳变守卫和 MAX_JUMP 限速三道兜住。
                    need_stable = STABLE_FRAMES_FIRST if held is None else 0
                    if stable_n >= need_stable and not jump_skip:
                        conf = int(best[4] * 100) if len(best) > 4 else 90
                        if conf > 100:
                            conf = 100
                        cxv = (x + w / 2.0) / RGB888P_SIZE[0] * 1000.0
                        cyv = (y + h / 2.0) / RGB888P_SIZE[1] * 1000.0
                        wv = w / RGB888P_SIZE[0] * 1000.0
                        hv = h / RGB888P_SIZE[1] * 1000.0
                        if REPORT_HMIRROR:
                            cxv = 1000.0 - cxv
                        if REPORT_VMIRROR:
                            cyv = 1000.0 - cyv
                        cxv = int(max(0, min(1000, cxv)))
                        cyv = int(max(0, min(1000, cyv)))
                        wv = int(max(0, min(1000, wv)))
                        hv = int(max(0, min(1000, hv)))
                        prev = None if held is None else [held[0], held[1]]
                        if held is None:
                            held = [cxv, cyv, wv, hv, conf]
                        else:
                            a = SMOOTH_ALPHA
                            held[0] = int(a * cxv + (1 - a) * held[0])
                            held[1] = int(a * cyv + (1 - a) * held[1])
                            held[2] = int(a * wv + (1 - a) * held[2])
                            held[3] = int(a * hv + (1 - a) * held[3])
                            held[4] = conf
                        if prev is not None and MAX_JUMP > 0:
                            for i in (0, 1):
                                d = held[i] - prev[i]
                                if d > MAX_JUMP:
                                    held[i] = prev[i] + MAX_JUMP
                                elif d < -MAX_JUMP:
                                    held[i] = prev[i] - MAX_JUMP
                        last_face_ms = now
                        DIAG[0] = "OK %d,%d %dx%d" % (cxv, cyv, wv, hv)
                        send_face(held)
                    elif jump_skip:
                        DIAG[0] = "JUMPHOLD %dx%d" % (bw, bh)
                    else:
                        DIAG[0] = "STAB %d/%d %dx%d" % (stable_n, need_stable, bw, bh)
                        if DEBUG and stable_n == 0:
                            log("unstable, skip")
            else:
                if last_face_ms and time.ticks_diff(now, last_face_ms) >= LOST_N_MS:
                    held = None
                    lock_cx = None
                    lock_cy = None
                    if time.ticks_diff(now, last_noface) >= noface_period:
                        last_noface = now
                        try:
                            sock.sendto(b"N\n", dest)
                            if MIRROR_TO_PC and MIRROR_IP:
                                sock.sendto(b"N\n", (MIRROR_IP, MIRROR_PORT))
                        except Exception as e:
                            log("sendto fail:", e)
                        if DEBUG:
                            print("N")
            # ★v47 诊断：串口死掉时唯一能看到 K230 内部状态的通道。
            # 每行以 '#' 开头，VOX3 侧会当调试行忽略（不参与人脸解析），
            # 但它的 V1036 日志会原样打印出来。
            if time.ticks_diff(now, diag_t[0]) >= 1000:
                diag_t[0] = now
                try:
                    # [v52] 每秒一条的诊断行现在自带 fps 与累计发包数。
                    #       串口是死的，主控 V1036 日志是唯一能看见 K230 内脏的地方：
                    #       有 fps 但 tx 不涨 = 发不出去（网络）；fps 掉到 3 以下 = 取帧/推理卡。
                    # [v53] 末尾四个数=每帧均值(ms)：gf=取帧 run=推理(含后处理)
                    #       post=其中后处理部分 dr=显示(已按 DISPLAY_EVERY 摊平)
                    _k = prof[3] if prof[3] > 0 else 1
                    _tg = prof[0] // _k // 1000
                    _tr = prof[1] // _k // 1000
                    _tp = PROF["post"] // _k // 1000
                    _td = prof[2] // _k // max(1, DISPLAY_EVERY) // 1000
                    sock.sendto(("#s=" + SCRIPT_VER
                                 + " %s n=%d fps=%.1f tx=%d T gf=%d run=%d post=%d dr=%d\n"
                                 % (DIAG[0], len(dets or []),
                                    cur_fps[0], tx_cnt[0],
                                    _tg, _tr, _tp, _td)).encode(), dest)
                    prof[0] = prof[1] = prof[2] = prof[3] = 0
                    PROF["post"] = 0
                except Exception:
                    pass
            # [v52] 显示降频：draw_result + show_image 是单帧最大开销，
            #       而且它只影响"人眼看着顺不顺"，完全不影响检测与发包。
            #       每 DISPLAY_EVERY 帧刷一次，省下的时间直接变成更高的跟踪帧率。
            display_n += 1
            if (display_n % DISPLAY_EVERY) == 0:
                _pd = _now_us()     # [v53] 显示计时开始
                try:
                    face_det.draw_result(pl, dets)
                except Exception as e:
                    log("draw fail:", e)
                try:
                    pl.show_image()
                except Exception as e:
                    # 原来这里裸奔，一抛异常就冒泡出去把整个脚本带走（画面定格）。
                    log("show_image fail:", e)
                prof[2] += time.ticks_diff(_now_us(), _pd)
            frames += 1
            prof[3] += 1
            if time.ticks_diff(time.ticks_ms(), fps_t) >= 5000:
                cur_fps[0] = frames * 1000.0 / 5000.0
                log("fps ~ %.1f, tx=%d, up %ds" % (cur_fps[0], tx_cnt[0],
                                                    time.ticks_diff(time.ticks_ms(), t0) // 1000))
                frames = 0
                fps_t = time.ticks_ms()
                net_keepalive()     # [v52] 掉网自愈
                gc.collect()
    except KeyboardInterrupt:
        log("stopped by user")
    except BaseException as e:
        sys.print_exception(e)
    finally:
        try:
            if face_det is not None:
                face_det.deinit()
        except Exception:
            pass
        try:
            if pl is not None:
                pl.destroy()
        except Exception:
            pass
        log("exit")
    if state["reload"]:
        log("machine.reset() -> reboot and refetch")
        try:
            import machine
            machine.reset()
        except Exception as e:
            log("reset fail:", e)
main()