Doing AGV that having goal to move by mission (from home to destination)

for wheel, i use motor that not have encoder so i put encoder bwtween motor gear and wheel gear, motor gear is 9 teeth, encoder gear also 9 teeth, wheel gear is 32 teeth, whel diameter 200mm

I connectt pi via ssh
-alias rpi="ssh yaman@192.168.137.50")
I mount pi folder to i can work in vm
-alias pimount='sshfs yaman@192.168.137.50:/home/yaman ~/pi'
-alias piumount='fusermount -u ~/pi'

hardware and Connection
-Raspi ubuntu24.04lts server
-Rplidar C1 USB to Raspi

-Arduino Mega
-Encoder(600P/Re6b2 cwz6c)
Left Wheel
-outA d2
-outB d3
Right Wheel
-outA d18
-outB d19
-Wheel Motor Driver(Cytron MDD20A)
Left
-pwm1 d6
-dir1 d52
Right
-pwm2 d7
-dir2 d53
-IMU(mpu9250)
-SCL d21
-SDA d20
