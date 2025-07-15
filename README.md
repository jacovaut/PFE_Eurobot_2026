# PFE_Eurobot_2026
<table>
  <tr>
    <td><img src="Logos/Logo_grum.png" alt="Grum Logo" height="200"></td>
    <td><img src="Logos/Logo_genie.png" alt="Genie Logo" height="200"></td>
  </tr>
</table>

### Tout le code pour le projet.

Svp créer un nouveau folder pour chaque partie et indiquer son utilité dans le tableau.


| Dir                | Purpose       |
| ------------------ |:------------------:|
| ws                 | main ros2 colcon workspace |
| devcontainer       | Code for docker containers |
| pio                | ESP32 Code      |

## Git
- Use comprehensive commit messages
- Don't be dumb

### To add a new feature
- `git checkout -b feature/some-new-feature`
- work and commit
- `git push origin feature/some-new-feature`
- Pull request in main


### First time setup
- To get code : `git clone https://github.com/jacovaut/PFE_Eurobot_2026.git`
- Username and email : 
```bash
git config --global user.name "Your Name"
git config --global user.email "youremail@yourdomain.com"
```

## Docker
to start a container :
- If on windows, use wsl
- Navigate to devcontainer
- `sudo chmod +x ./Start-container.bash`
- `sudo ./Start-contaier.bash`


## Ros2

Example Ros2 workspace structure :
```
ws
├───build (ignore)
├───install (Running files are in here)
├───log
└───src (Code)
    ├───package_1 (Example packages (use git submodules))
    ├───package_2
    └───pfe (main code)
        ├───config (.yaml config files)
        ├───description (robot description and rviz files)
        │   ├───rviz
        │   └───urdf
        ├───launch (all launch files)
        ├───maps
        ├───resource
        ├───test
        ├───pfe (custom nodes)
        └───worlds (gazebo worlds)
```
The build, install and log folders should be in ignored by git.

For premade packages, use sudo apt install ros-jazzy-PACKAGE_NAME or git submodules


- To build the workspace
    - cd ~/PFE_EUROBOT_2026/ws
    - colcon build --symlink-install
- To build a specific package
    - cd ~/PFE_EUROBOT_2026/ws
    - colcon build --symlink-install --packages-select PACKAGE
### Ros2 packages

|Package|Apt/Submodule|
|---|----|
|nav2|apt|
|Gazebo|apt|
|rviz|apt|
|xacro|apt|
|joint state publisher|apt|
|teleop twist keyboard|apt|



## TO DO
- [x] Configure Docker
- [x] README
- [ ] Add pio project
- [x] Configure Ros2 ws
- Packages
    - [ ] nav2
    - [ ] Robot localization
    - [ ] Lidar
    - [ ] joy

![logo](Logos/stock-photo-fractals-background-owl-portrait-animal-1703663572%20(1).jpg)
