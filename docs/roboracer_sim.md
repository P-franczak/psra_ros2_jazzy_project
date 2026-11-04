# Własny pakiet z symulacją samochodu: Gazebo i RViz

Ta instrukcja prowadzi przez utworzenie własnego pakietu ROS 2 `roboracer_sim`, który uruchamia samochód w Gazebo i pokazuje go w RViz.

Model samochodu jest w osobnym pakiecie `roboracer_description` (osobne repozytorium, pobierane do `src/vcs_repos/`). Twój pakiet go **używa**, a nie kopiuje:

```
roboracer_description   model: xacro, siatki, podgląd w RViz       (wspólne repozytorium)
        ▲ exec_depend
roboracer_sim           symulacja: świat, mostek ROS↔Gazebo, launch (Twój pakiet)
```

Wszystkie polecenia wykonuj w terminalu kontenera (VS Code, **Reopen in Container**).

## 0. Przygotuj workspace i sprawdź gotowy przykład

```bash
bash ~/ros2_ws/src/.devcontainer/setup.sh   # pobiera roboracer_description (vcs) i instaluje zależności (rosdep)
cd ~/ros2_ws && ./build.sh && source install/setup.bash

ros2 launch roboracer_description display.launch.py   # sam model w RViz, bez symulacji
```

Jeśli to działa, środowisko jest gotowe.

## 1. Utwórz pakiet

```bash
cd ~/ros2_ws/src
ros2 pkg create roboracer_sim --build-type ament_python --license Apache-2.0
cd roboracer_sim
mkdir -p launch config worlds urdf
```

Powstanie:

```
roboracer_sim/
├── package.xml          opis pakietu i jego zależności
├── setup.py             co i gdzie instalować
├── setup.cfg
├── roboracer_sim/       moduł Pythona (Twoje węzły)
├── resource/
├── test/
└── launch/  config/  worlds/  urdf/   ← utworzone przez mkdir (pliki napiszesz w kroku 4, urdf/ przyda się w kroku 7)
```

## 2. Dodaj zależności w `package.xml`

Dopisz przed `<test_depend>`:

```xml
<exec_depend>roboracer_description</exec_depend>   <!-- model samochodu -->
<exec_depend>xacro</exec_depend>                   <!-- przetwarzanie modelu xacro -->
<exec_depend>robot_state_publisher</exec_depend>   <!-- TF z modelu -->
<exec_depend>ros_gz_sim</exec_depend>              <!-- Gazebo i spawn robota -->
<exec_depend>ros_gz_bridge</exec_depend>           <!-- mostek tematów ROS <-> Gazebo -->
<exec_depend>rviz2</exec_depend>
<exec_depend>teleop_twist_keyboard</exec_depend>   <!-- sterowanie z klawiatury -->
```

Zainstaluj brakujące paczki:

```bash
cd ~/ros2_ws
rosdep install -i --from-paths src --rosdistro $ROS_DISTRO -y
```

Szczegóły działania rosdep są w [rosdep.md](rosdep.md).

## 3. Zainstaluj pliki pakietu w `setup.py`

`colcon build` kopiuje do `install/` tylko pliki wymienione w `data_files`. Bez tego `ros2 launch` nie znajdzie Twoich plików. Na górze `setup.py` dodaj importy:

```python
import os
from glob import glob
```

a listę `data_files` uzupełnij tak:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
],
```

## 4. Napisz pliki symulacji

Symulacja składa się z trzech plików, które piszesz w swoim pakiecie:

| Plik | Co opisuje |
|------|------------|
| `worlds/track.sdf` | świat Gazebo: fizyka, światło, podłoże, ściany toru |
| `config/bridge_config.yaml` | które tematy przechodzą między Gazebo a ROS |
| `launch/simulation.launch.py` | co uruchomić: Gazebo, robota, mostek, RViz |

Model samochodu i konfiguracja RViz pochodzą z `roboracer_description`, więc ich nie piszesz.

### 4.1. Świat: `worlds/track.sdf`

Świat Gazebo to plik SDF. Najważniejsze są **wtyczki systemowe**. Bez nich świat się wczyta, ale robot nie będzie działał:

| Wtyczka | Bez niej |
|---------|----------|
| `Physics` | nic się nie porusza |
| `UserCommands` | nie da się dodać robota (`ros_gz_sim create` kończy się błędem) |
| `SceneBroadcaster` | okno Gazebo jest puste |
| `Sensors` (silnik `ogre2`) | lidar i kamera nie publikują danych |

Utwórz `worlds/track.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="track">

    <!-- Wtyczki systemowe -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Słońce -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Podłoże 20 x 20 m -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Tor: prostokąt 10 x 6 m z wysepką 6 x 2 m w środku -->
    <model name="walls">
      <static>true</static>
      <link name="north">
        <pose>0 3 0.15 0 0 0</pose>
        <collision name="c"><geometry><box><size>10 0.1 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>10 0.1 0.3</size></box></geometry></visual>
      </link>
      <link name="south">
        <pose>0 -3 0.15 0 0 0</pose>
        <collision name="c"><geometry><box><size>10 0.1 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>10 0.1 0.3</size></box></geometry></visual>
      </link>
      <link name="east">
        <pose>5 0 0.15 0 0 0</pose>
        <collision name="c"><geometry><box><size>0.1 6 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.1 6 0.3</size></box></geometry></visual>
      </link>
      <link name="west">
        <pose>-5 0 0.15 0 0 0</pose>
        <collision name="c"><geometry><box><size>0.1 6 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.1 6 0.3</size></box></geometry></visual>
      </link>
      <link name="island">
        <pose>0 0 0.15 0 0 0</pose>
        <collision name="c"><geometry><box><size>6 2 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>6 2 0.3</size></box></geometry></visual>
      </link>
    </model>

  </world>
</sdf>
```

Uwagi:
- `<pose>x y z roll pitch yaw</pose>` to położenie w metrach i obrót w radianach. Ściany mają wysokość 0.3 m, więc ich środek jest na z = 0.15.
- Wymiary podaje się w elementach (`<box><size>10 0.1 0.3</size></box>`), nie w atrybutach (`<box size="..."/>`). Atrybuty Gazebo ignoruje z ostrzeżeniem `not defined in SDF`.
- Ściany muszą być wyższe niż płaszczyzna skanu lidaru (ok. 0.23 m nad ziemią), inaczej lidar ich nie zobaczy.
- `<static>true</static>` oznacza, że obiekt się nie porusza i nie podlega fizyce.

Sprawdź poprawność pliku i obejrzyj świat bez robota:

```bash
cd ~/ros2_ws/src/roboracer_sim
gz sdf -k worlds/track.sdf     # oczekiwane: Valid.
gz sim worlds/track.sdf
```

### 4.2. Mostek: `config/bridge_config.yaml`

Gazebo ma własne tematy (`gz topic -l`), niewidoczne w ROS (`ros2 topic list`). Mostek `ros_gz_bridge` przenosi wybrane tematy na drugą stronę, z konwersją typu wiadomości. Każdy wpis w pliku to jeden temat i ma pięć pól:

| Pole | Znaczenie |
|------|-----------|
| `ros_topic_name` | nazwa tematu w ROS |
| `gz_topic_name` | nazwa tematu w Gazebo (nadaje ją czujnik albo wtyczka w modelu) |
| `ros_type_name` | typ wiadomości ROS, np. `sensor_msgs/msg/LaserScan` |
| `gz_type_name` | odpowiadający mu typ Gazebo, np. `gz.msgs.LaserScan` |
| `direction` | `GZ_TO_ROS` (dane z symulacji), `ROS_TO_GZ` (polecenia do symulacji) albo `BIDIRECTIONAL` |

**Zadanie:** uzupełnij plik tak, żeby do ROS trafiały wszystkie dane potrzebne do jazdy i wizualizacji. Wpis `/clock` jest kompletny i służy za wzór. W pozostałych zastąp każde `___` właściwą wartością.

Utwórz `config/bridge_config.yaml`:

```yaml
# WZÓR: czas symulacji (bez niego węzły z use_sim_time: true stoją w miejscu)
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

# 1. Sterowanie samochodem: prędkość liniowa i kątowa wysyłana z ROS (np. przez teleop)
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 2. Odometria: pozycja i prędkość samochodu liczona przez wtyczkę napędu
- ros_topic_name: "/odom"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 3. Transformacja odom -> base_footprint publikowana przez wtyczkę napędu
- ros_topic_name: "/tf"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 4. Kąty kół i zwrotnic (robot_state_publisher liczy z nich TF kół)
- ros_topic_name: "/joint_states"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 5. Lidar
- ros_topic_name: "/scan"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 6. Obraz z kamery kolorowej
- ros_topic_name: "/camera/color/image_raw"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 7. Parametry kamery kolorowej (ogniskowa, rozdzielczość)
- ros_topic_name: "/camera/color/camera_info"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___

# 8. Chmura punktów z kamery głębi
- ros_topic_name: "/camera/depth/points"
  gz_topic_name: "___"
  ros_type_name: "___"
  gz_type_name: "___"
  direction: ___
```

**Skąd wziąć dane:**

1. **Nazwy tematów Gazebo.** Uruchom symulację (krok 5) nawet z niekompletnym mostkiem i wypisz tematy Gazebo:
   ```bash
   gz topic -l
   ```
   Nazwy nadaje model, w elementach `<topic>` czujników i wtyczek w `roboracer_description/urdf/roboracer.urdf.xacro`. Kamera kolorowa publikuje swoje parametry na temacie obok obrazu, z końcówką `camera_info`.
2. **Typ Gazebo.** Dla konkretnego tematu:
   ```bash
   gz topic -i -t /scan          # pokazuje m.in. typ wiadomości, np. gz.msgs.LaserScan
   ```
3. **Typ ROS.** Para do typu Gazebo jest w tabeli w [ros_gz_bridge.md](ros_gz_bridge.md). Niektóre pary są nieoczywiste: stany przegubów przychodzą z Gazebo jako `gz.msgs.Model`, a transformacje jako `gz.msgs.Pose_V`.
4. **Kierunek.** Zastanów się, kto jest źródłem danych. Czujniki i wtyczki w symulacji wysyłają dane do ROS, a polecenia jazdy płyną z ROS do symulacji.

**Wydajność kamery.** Do wpisów 6–8 dopisz `lazy: true`. Mostek subskrybuje wtedy temat w Gazebo dopiero wtedy, gdy ktoś w ROS go odbiera. Gazebo renderuje kamerę tylko dla odbiorców, więc bez tego cały czas liczy obraz i chmurę punktów, nawet gdy nikt na nie nie patrzy.

**Sprawdzenie** (przy działającej symulacji):

| Test | Polecenie | Oczekiwany wynik |
|------|-----------|------------------|
| wszystkie tematy w ROS | `ros2 topic list` | 9 tematów z pliku |
| lidar działa | `ros2 topic hz /scan` | ok. 10 Hz |
| ramka lidaru | `ros2 topic echo --once /scan --field header.frame_id` | `laser` |
| sterowanie | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | samochód jedzie w Gazebo |
| odometria | `ros2 topic echo /odom --field pose.pose.position` | pozycja zmienia się podczas jazdy |
| koła w RViz | RViz, display *RobotModel* | brak błędów *No transform* przy kołach |
| kamera | `ros2 topic hz /camera/color/image_raw` | ok. 15 Hz |

Jeśli mostek nie startuje albo pomija wpis, przeczytaj jego komunikaty w terminalu z `ros2 launch`: nieznany typ albo literówka w nazwie typu jest tam zgłaszana wprost.

### 4.3. Launch: `launch/simulation.launch.py`

Plik launch to skrypt Pythona z funkcją `generate_launch_description()`. Zwraca ona listę akcji do wykonania: argumenty, węzły i dołączone inne pliki launch. Utwórz `launch/simulation.launch.py` i wpisuj go po kolei, fragment po fragmencie. Wszystkie fragmenty razem dają cały plik.

**Importy.**

```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
```

**Ścieżki do plików.** `get_package_share_directory` zwraca katalog `install/share/<pakiet>`, do którego `colcon build` skopiował pliki (dlatego w kroku 3 były wpisy w `data_files`):

```python


def generate_launch_description():
    pkg_dir = get_package_share_directory('roboracer_sim')
    description_dir = get_package_share_directory('roboracer_description')

    world_file = os.path.join(pkg_dir, 'worlds', 'track.sdf')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    xacro_file = os.path.join(description_dir, 'urdf', 'roboracer.urdf.xacro')
    rviz_config = os.path.join(description_dir, 'rviz', 'roboracer.rviz')
```

**Argumenty.** `DeclareLaunchArgument` pozwala zmieniać ustawienia z linii poleceń (`body:=open`). Wartość odczytuje się przez `LaunchConfiguration('nazwa')`:

```python

    body_arg = DeclareLaunchArgument('body', default_value='shell', choices=['shell', 'open'])
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
```

**Opis robota.** `Command` uruchamia `xacro` w chwili startu i zamienia model na URDF. `ParameterValue(..., value_type=str)` mówi ROS, że wynik to tekst, a nie YAML:

```python

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' body:=', LaunchConfiguration('body')]),
        value_type=str
    )
```

**robot_state_publisher.** Węzeł dostaje opis robota, publikuje go na `/robot_description` i liczy TF wszystkich części. `use_sim_time: True` każe mu używać czasu z Gazebo (`/clock`):

```python

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )
```

**Gazebo.** Zamiast uruchamiać `gz sim` bezpośrednio, dołączasz gotowy launch z `ros_gz_sim`. Ustawia on ścieżki, dzięki którym Gazebo znajduje siatki z `package://roboracer_description/...`. Opcja `-r` od razu startuje symulację:

```python

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )
```

**Robot w świecie.** `create` czyta opis z tematu `/robot_description` i dodaje robota do Gazebo w zadanym miejscu. Pozycja (0, −2) to dolny korytarz toru z kroku 4.1, a `-z 0.05` to 5 cm nad ziemią, żeby koła nie zaczynały wewnątrz podłoża:

```python

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'roboracer',
                   '-x', '0.0', '-y', '-2.0', '-z', '0.05'],
        output='screen'
    )
```

**Mostek.** Ten sam węzeł obsłuży wszystkie tematy z pliku z kroku 4.2:

```python

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )
```

**RViz.** Konfiguracja pochodzi z `roboracer_description`. Flaga `-f odom` ustawia *Fixed Frame* na układ odometrii, a `condition` sprawia, że RViz startuje tylko przy `rviz:=true`:

```python

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config, '-f', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
```

**Lista akcji.** Kolejność w liście nie oznacza kolejności startu, bo wszystko rusza równolegle. `create` czeka jednak, aż pojawi się `/robot_description` i świat Gazebo:

```python

    return LaunchDescription([
        body_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        rviz,
    ])
```

Sprawdź, czy plik jest poprawny, bez uruchamiania symulacji (po zbudowaniu pakietu w kroku 5):

```bash
ros2 launch roboracer_sim simulation.launch.py --show-args    # powinien pokazać body i rviz
```

## 5. Zbuduj i uruchom

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select roboracer_sim
source install/setup.bash

ros2 launch roboracer_sim simulation.launch.py
```

Dzięki `--symlink-install` zmiany w plikach `.xacro`, `.yaml` i `.launch.py` działają bez ponownego budowania. Nowe pliki wymagają jednak ponownego `colcon build`.

Argumenty (`ros2 launch roboracer_sim simulation.launch.py --show-args`):

| Argument | Domyślnie | Znaczenie |
|----------|-----------|-----------|
| `body`   | `shell`   | `shell` = karoseria F1TENTH, `open` = otwarte podwozie z elektroniką |
| `rviz`   | `true`    | czy uruchomić RViz |

Przykład: `ros2 launch roboracer_sim simulation.launch.py body:=open rviz:=false`

### Model w RViz bez symulacji

```bash
ros2 launch roboracer_description display.launch.py
```

Okno **Joint State Publisher** ma suwaki, którymi skręcasz koła (`front_*_steering_joint`) i je obracasz (`*_wheel_joint`). Tak najszybciej sprawdzisz, czy model wygląda poprawnie i czy przeguby kręcą się we właściwą stronę.

### Jazda

W drugim terminalu:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Klawisze `i` (przód), `,` (tył), `j`/`l` (skręt), `k` (stop). Okno terminala z teleop musi być aktywne.

Bez klawiatury możesz wysłać stałą komendę:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.3}}'
```

## 6. Co jest w RViz

| Display | Temat | Co pokazuje |
|---------|-------|-------------|
| RobotModel  | `/robot_description` | model samochodu |
| LaserScan   | `/scan` | punkty z lidaru (czerwone) |
| Odometry    | `/odom` | strzałki z historią pozycji |
| DepthPoints | `/camera/depth/points` | chmura punktów z kamery głębi (domyślnie wyłączona) |
| ColorImage  | `/camera/color/image_raw` | obraz z kamery (domyślnie wyłączony) |
| TF          | – | wszystkie ramki (domyślnie wyłączony) |

Displaye włączasz i wyłączasz checkboxem w panelu **Displays**. Nowe dodajesz przyciskiem **Add** → zakładka **By topic**.

**Fixed Frame** (w *Global Options*) to układ, względem którego RViz rysuje wszystko:
- w symulacji `odom`, więc samochód jeździ po siatce,
- w `display.launch.py` `base_footprint`, bo bez symulacji nie ma odometrii.

Własną konfigurację RViz zapisz przez **File → Save Config As** w swoim pakiecie, np. `~/ros2_ws/src/roboracer_sim/config/my.rviz`. Następnie w `simulation.launch.py` ustaw `rviz_config = os.path.join(pkg_dir, 'config', 'my.rviz')`.

## 7. Własna wersja modelu

Nie edytuj plików w `src/vcs_repos/roboracer_description`. To wspólne repozytorium: zmiany zginą przy aktualizacji i nie trafią do Twojego repozytorium. Zamiast tego skopiuj model do swojego pakietu:

```bash
cd ~/ros2_ws/src/roboracer_sim
cp ../vcs_repos/roboracer_description/urdf/roboracer.urdf.xacro urdf/

# simulation.launch.py: model z Twojego pakietu zamiast z roboracer_description
sed -i "s/xacro_file = os.path.join(description_dir, /xacro_file = os.path.join(pkg_dir, /" launch/simulation.launch.py

cd ~/ros2_ws && colcon build --symlink-install --packages-select roboracer_sim
```

Siatki (`package://roboracer_description/meshes/...`) i kamera nadal pochodzą z zależności, więc kopiujesz tylko jeden plik.

Najważniejsze parametry są na początku `urdf/roboracer.urdf.xacro`:

```xml
<xacro:property name="wheelbase" value="0.325"/>     <!-- rozstaw osi -->
<xacro:property name="track_width" value="0.245"/>   <!-- rozstaw kół -->
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="steering_limit" value="0.42"/> <!-- maks. skręt [rad] -->
<xacro:property name="lidar_height" value="0.175"/>
```

Reszta modelu (pozycje kół, lidaru, kamery, parametry napędu Ackermann) jest liczona z tych wartości, więc zmieniasz je w jednym miejscu.

> [!IMPORTANT]
> Karoseria `shell` to siatka o stałych wymiarach, pasująca do `wheelbase` 0.325 i `track_width` 0.245. Po zmianie tych wartości uruchamiaj z `body:=open`, bo wtedy podwozie dopasowuje się do parametrów.

Budowa modelu:

| Część | Gdzie w pliku | Uwagi |
|-------|---------------|-------|
| `base_footprint` → `base_link` | początek | `base_footprint` na ziemi, `base_link` w środku tylnej osi |
| Koła | makra `rear_wheel`, `front_wheel` | przód: zwrotnica (`revolute`, oś Z) + koło (`continuous`, oś Y) |
| Napęd | `gz-sim-ackermann-steering-system` | czyta `/cmd_vel`, publikuje `/odom` i TF `odom → base_footprint` |
| Lidar | link `laser`, czujnik `gpu_lidar` | temat `/scan` |
| Kamera D435 | makro `sensor_d435` + 2 czujniki | obraz `/camera/color/image_raw`, chmura `/camera/depth/points` |
| Elementy ozdobne | makra `vbox`, `vcyl` | każdy to osobny link (RViz koloruje cały link jednym kolorem) |

Po zmianie modelu najpierw sprawdź, czy się przetwarza, a dopiero potem uruchom symulację:

```bash
xacro ~/ros2_ws/src/roboracer_sim/urdf/roboracer.urdf.xacro > /tmp/roboracer.urdf && check_urdf /tmp/roboracer.urdf
```

### Nowy czujnik albo temat

Czujnik dodajesz w xacro w bloku `<gazebo reference="nazwa_linku">`. Jego temat z Gazebo trzeba przekazać do ROS w `config/bridge_config.yaml`:

```yaml
- ros_topic_name: "/moj_temat"
  gz_topic_name: "/moj_temat"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

Pary typów ROS ↔ Gazebo są opisane w [ros_gz_bridge.md](ros_gz_bridge.md), a ramki czujników w [tf.md](tf.md). Listę tematów Gazebo pokaże `gz topic -l`, a tematów ROS `ros2 topic list`.

## 8. Najczęstsze problemy

| Objaw | Rozwiązanie |
|-------|-------------|
| `Package 'roboracer_sim' not found` | Nie wykonano `source install/setup.bash` po budowaniu (albo budowanie się nie udało). |
| `Package 'roboracer_description' not found` | Nie pobrano repozytorium modelu. Uruchom `bash ~/ros2_ws/src/.devcontainer/setup.sh` (albo `vcs import --skip-existing src < src/roboracer.repos` w `~/ros2_ws`) i zbuduj workspace. |
| `Multiple packages found with the same name` | Dwa pakiety o tej samej nazwie w `src/`, np. Twój pakiet nazwany `roboracer_description`. Zmień nazwę swojego pakietu. |
| `file not found` dla `.yaml`/`.sdf`/`.xacro` | Katalogu brakuje w `data_files` w `setup.py`. Popraw i zbuduj ponownie. |
| Model w RViz jest biały albo `No transform from [...]` | Brak `robot_state_publisher` albo stanów przegubów: w symulacji potrzebny jest mostek `/joint_states`. |
| Robot nie reaguje na `/cmd_vel` | Sprawdź `ros2 topic info /cmd_vel`: subskrybentem musi być `ros_gz_bridge`. Teleop musi mieć aktywne okno terminala. |
| W Gazebo brakuje kół lub karoserii (`Unable to find file`) | `roboracer_description` nie jest zbudowany albo Gazebo nie zostało uruchomione przez `ros_gz_sim/launch/gz_sim.launch.py`. |
| W RViz brak danych z czujników, a `ros2 topic hz /scan` nic nie pokazuje | Temat nie jest w `bridge_config.yaml` albo nazwy tematów w xacro i w mostku się różnią. |
| Dwie osoby na jednym komputerze widzą swoje roboty | Ustaw różne `ROS_DOMAIN_ID` i `GZ_PARTITION` (patrz README). |
