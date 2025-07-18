import serial
import serial.tools.list_ports
import time
import socket
import colorsys


time.sleep(20)
# --------------------------------------------------------
# Получение IP адреса устройства
# --------------------------------------------------------
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(('10.254.254.254', 1))  # Пытаемся соединиться с любым внешним адресом
        ip = s.getsockname()[0]
    except:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

# --------------------------------------------------------
# Преобразование IP в команду для светодиодов
# --------------------------------------------------------
def color_to_4bits(r8, g8, b8, brightness):
    """
    Преобразуем 8-битные R,G,B (0..255) и яркость (0..15)
    в 16-битное число с упаковкой: [R(4 бита), G(4 бита), B(4 бита), Br(4 бита)].
    Возвращаем 4-символьную HEX-строку, например "0FAB".
    """
    r4 = (r8 * 15) // 255
    g4 = (g8 * 15) // 255
    b4 = (b8 * 15) // 255
    
    val = (r4 << 12) | (g4 << 8) | (b4 << 4) | (brightness & 0x0F)
    return f"{val:04X}"
    
def hsv_to_rgb_4bit(h, s, v, brightness):
    """
    Преобразует HSV (0..1) в 4-битный RGB-формат.
    """
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    r8, g8, b8 = int(r * 255), int(g * 255), int(b * 255)
    return color_to_4bits(r8, g8, b8, brightness)
    
def ip_to_led_command(ip):
    ip_parts = int(ip.split('.')[-1])  # Берем последние 3 цифры IP
    data_blocks = [hsv_to_rgb_4bit(0.3, 1, 1, 5) for i in range(ip_parts%10)] + [hsv_to_rgb_4bit(0, 1, 1, 5)] + [hsv_to_rgb_4bit(0.6, 1, 1, 5) for i in range(ip_parts//10%10)] + [hsv_to_rgb_4bit(0, 1, 1, 5)] + [hsv_to_rgb_4bit(0.8, 1, 1, 5) for i in range(ip_parts//100%10)]
    hex_data = "".join(data_blocks)
    print(data_blocks)
    command = f"SET_LED_DATA {len(data_blocks)} {hex_data}\n"
    ser.write(command.encode('ascii'))


ser = serial.Serial("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0", 115200, timeout=1)
ser.write(b"PING\n")
ip = get_ip_address()
print(f"IP адрес устройства: {ip}")

ip_to_led_command(ip)
ser.close()