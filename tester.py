import serial


test = ["94","c3","22","9F","AA","6E"]
lent = len(test)
print(lent)
for i,b in test: print(f"{i} of {b}")

try:
    ser = serial.Serial("COM4",115200, timeout=1)
    print("Connected")
except serial.SerialException :
    ser = None

def pars(cmd:str)->bytes:
    user_in = cmd.strip().replace(" ","").upper()
    if user_in.startswith("0X"):
        user_in = user_in[2:]
    
    length = len(user_in) // 2
    frame = []
    frame.append(length)

    for i in range(0, len(user_in), 2):
        frame.append(int(user_in[i:i+2], 16))


    while len(frame) < 8:
        frame.append(0xAA)

    return bytes(frame)

print("tester : ")

while True:

    if ser is None :
        print("COM4 not Connected")
        break
    try:
        cmd = input(">")
        frame = pars(cmd)
        print("Frame:", frame)
        ser.write(frame)
        print(ser.read(8).hex().upper())
    except serial.SerialException:
        print("COM4 Lost")
        ser = None
        