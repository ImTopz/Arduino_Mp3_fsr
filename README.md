# Arduino_MP3_FSR
一款通过压力传感器控制灯Arduino传感器


开机后，自动进入轮询模式，通过FSR薄膜压力传感器来控制LED灯圈的亮灭和MP3或者WAV格式音乐的播放，播放的指令为串口操作，如果第一次检测到压力，则LED灯圈亮，MP3不播放，第二次则MP3播放，LED灯圈灭，第三次等同于第一次，一共支持三个FSR传感器和灯圈，灯圈采用的WS2812的库，轮询检测


PIR.INO是通过PIR传感器控制LED灯圈的亮灭、通过倾斜开关来实现音乐的切换。

OS1生成成功

回顾当时的代码感觉最好的实现方式应该是通过中断来实现，这样可以极大的增强代码的鲁棒性，有时间会补写中断实现的代码


An Arduino-based mp3 device with pressure sensor to control play and pause
After starting up, it will automatically enter the polling mode, and control the on and off of the LED light circle and the playback of MP3 or WAV format music through the FSR film pressure sensor. On, MP3 is not playing, MP3 is playing for the second time, the LED light circle is off, the third time is equal to the first time, a total of three FSR sensors and light circles are supported, the light circle uses WS2812 library, polling detection


PIR.INO uses the PIR sensor to control the LED light ring to turn on and off, and the music switch is realized by tilting the switch.

OS1 generated successfully

Looking back at the code at that time, I feel that the best way to implement it should be through interrupts, which can greatly enhance the robustness of the code, and I will rewrite the code for interrupt implementation when I have time
