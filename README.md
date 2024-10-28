

# /home/pi

[wiki](https://github.com/GMU-ASRC/GMU-ASRC/wiki)

We manage the software on our fleet of Hiwonder TurboPi robots with this git repo.  
Different behaviors are stored in git branches.

### Guides:
* Install this to a pi: [Turbopi Git Syncing](https://github.com/GMU-ASRC/GMU-ASRC/wiki/Turbopi-Git-Syncing)


## Manufacturer Resources:  
[Assembly Guide](https://drive.google.com/drive/folders/1x5IXJTTF_mz3FnN-iiRB4MxgKbYnIj25?usp=sharing)  
[Source Code + System Image](https://drive.google.com/drive/folders/1Y9FdmRe_h6JPQ0ggJe8bi1GDT4IsESSV?usp=sharing)  
[Assembly Video](https://www.youtube.com/playlist?list=PLFbzd0m6AcmLzo53o2Tsa20BS350rWGMj)  
[App links](https://www.hiwonder.com.cn/download/foreignSofts?get_app=1) | [iOS direct link](https://apps.apple.com/cn/app/wonderpi/id1477946178) |  [Android direct link](https://play.google.com/store/apps/details?id=com.Wonder.Pi)  

## Other resources
* [**VNC Viewer by RealVNC**](https://www.realvnc.com/en/connect/download/viewer/)


## Directory Structure

```
├── boot/             # buttonman, battery checker, and lobot services
├── caspyan/          # python-based Spiking Neural Network processor
├── hiwonder-common/  # python package for common code
|   drive.py          # turn on the motors at a specific speed. see below for usage
|   program1.sh       # buttonman starter script. 1 click of button 2 will run this.
```

<details>
  
  <summary> <h2> <code>drive.py</code> quick guide </h2> </summary>
  
  run with `sudo python drive.py` or `sudo ./drive.py`  
  By default, it runs the equivalent of `mecanum.set_chassis(0, 90, 0)`, i.e. `v, d, w = 0, 90, 0`  
  ``` console
  sudo python drive.py -v 100 -d 90 -w 0.5
  ```
  **Arguments**:
  * `-v` : int ∈ **\[-100, 100\]**, default: **0**  
        Speed. Not linear, does not correspond to physical speed.
  * `-d` : float ∈ **\[-360, 360\]**, default: **90**  
        Direction (deg). 0 degrees is strafing right, 90 forwards.
  * `-w` : float ∈ **\[-2.0, 2.0\]**, default: **0**  
        Turning rate. Not linear, does not correspond to physical angular velocity.

  ***
</details>



<details>
  
  <summary> <h2> <code>milling_controller.py</code> quick guide </h2> </summary>

  If started simultaneously on 6 TurboPis arranged in a tight circle, the robots will mill in a circle.
  
  Immediately start milling: 
  ``` console
  sudo python milling_controller.py
  ```

  Wait for a command or key1 press to actually start: 
  ``` console
  sudo python milling_controller.py --sta
  ```

  Run the program without moving the wheels, and collect the logs in `~/logs/test_run`: 
  ``` console
  sudo python milling_controller.py test_run --dry
  ```

  **Arguments**:
  * `project` : **string**, default is of the format YYMMDD-hhmmss-MillingProgram-turbopi-nn  
        This will be the name of the folder used for logs.
  * `--root` : **string**, default: **`'~/logs/'`**  
        This is where the project folder will be stored. If the directory does not exist, you will be prompted to create it.
  * `--dry_run` : **store_true flag**  
        Wheel motors will not actuate if this is passed.
  * `--start_paused` : **store_true flag**  
        The robot will start in a paused state.
        You can start the program with 1 click of key1 (towards the front of the robot) or by using the [remote control broadcaster](https://github.com/GMU-ASRC/GMU-ASRC/blob/main/hiwonder/turbopi/remote_switcher.py).
  * `--nolog` : **store_true flag**  
        Logging of will be disabled if this is passed. By default, sensor and wheel movements are logged on each loop/frame of the program.

  ***
</details>
