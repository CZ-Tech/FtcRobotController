## 一个事件驱动架构的FTC机器人控制程序框架
（没做呢）

由Gemini编写

> 本图表阐述了一个为FTC机器人设计的高性能、事件驱动的软件架构，旨在最大化利用REV控制中心（Control Hub）的四核(待确认)CPU。
> 该架构由三个并行运行并通过事件驱动架构通信的主要线程组成。

## 主要线程架构

| 线程 1: 传感器                         | 线程 2: 底盘驱动                  | 线程 3: 视觉               |
|:----------------------------------|:----------------------------|:-----------------------|
| **(高优先级)** - *生产者*                | **(最高优先级)** - *主要消费者 & 分发器* | **(普通/低优先级)** - *生产者*  |
| - 以极高频率循环 (200-800Hz)             | - 机器人主“心跳”循环 (100Hz左右)      | - 以较低频率循环 (看情况Hz ;)    |
| - 调用I2C读取指令 (`pinpoint.update()`) |                             | - 从图像处理单元读取信息          |
| - 将融合后的`Pose2D`发布到共享队列            | - 消费最新的传感器/手柄/视觉数据          | - 运行一些算法               |
|                                   | - 直接读取手柄状态                  | - 根据计算结果调用底盘驱动api等设置目标 |
|                                   | - 计算电机功率 (PID/自适应功率控制)      |                        |
| - 准备并发送遥测数据                       |                             |                        |
| **数据流出:** `Pose2D` 对象、速度信息        | **数据流入:** 传感器/视觉/手柄数据       | **数据流出:** 数据对象         |
| --- 通过 原子变量引用 →                   | (消费数据)                      | ↓ 通过消息队列通知主线程  ---     |

<br>

## 数据流说明

1.  **[传感器线程]:** 持续轮询Pinpoint(I2C)，获取新的位置、速度信息，并将其放入一个线程安全的数据对象中。这是它的唯一职责。

2.  **[视觉线程]:** 独立地处理摄像头图像。当它发现一个有效目标时，就会将目标的位置信息放入一个消息队列。
                    主类读取并根据需要启动一些任务流程

3.  **[核心线程]:** 循环，在每个周期执行以下操作：
    *   **a.** 轮询来自各种传感器线程的信息，以获取最新的机器人位姿。
    *   **d.** 根据当前设定的目标位置和当前位置和速度计算个电机功率并设定输出功率。

---

以下是ftc sdk官方readme



## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""