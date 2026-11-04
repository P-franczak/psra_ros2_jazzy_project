# TF dla kamery i lidaru w symulacji Gazebo + ROS 2

Instrukcja opisuje, jak poprawnie dodać ramki TF czujników robota symulowanego w Gazebo Harmonic. Przykładem jest pojazd z kamerą RealSense D435 i lidarem RPLIDAR C1 (`simple_example`, launch `vehicle_sensors.launch.py`).

## Po co jest TF

Każdy czujnik publikuje dane we własnym układzie współrzędnych, wskazanym w polu `header.frame_id`. Chmura punktów z kamery jest w `camera_link`, a skan lidaru w `lidar_link`. Żeby RViz albo algorytm (np. SLAM) mógł połączyć te dane z robotem, musi znać położenie każdej ramki czujnika względem korpusu. Tę wiedzę dostarcza TF. Ramki przymocowane na stałe trafiają na `/tf_static`, a ruchome (np. koła) na `/tf`.

Każda ramka ma **dokładnie jednego rodzica**, a każdą transformację publikuje jedno źródło.

## Osie (REP-103)

| Ramka | x | y | z |
|---|---|---|---|
| `chassis`, `camera_link`, `lidar_link` | do przodu | w lewo | w górę |
| ramka optyczna kamery (`*_optical_frame`) | w prawo | w dół | do przodu |

- Układy są prawoskrętne. Kąty: roll wokół x, pitch wokół y, yaw wokół z. Jednostki: metry i radiany.
- **Lidar (`sensor_msgs/LaserScan`):** kąt 0 to oś +x (przód), kąty rosną przeciwnie do ruchu wskazówek zegara (w stronę +y, w lewo). W tym modelu skan obejmuje zakres od `angle_min = -π` (tył) przez 0 (przód) do `angle_max ≈ +π`.
- **Kamera z Gazebo:** chmura punktów jest w konwencji linku (x do przodu). Obrazy w ROS zwykle zakłada się w ramce optycznej (patrz [Ramka optyczna kamery](#ramka-optyczna-kamery)).

## Drzewo TF w tym projekcie

```
chassis
├── camera_link   (RealSense D435)   ← robot_state_publisher, /tf_static
├── lidar_link    (RPLIDAR C1)       ← robot_state_publisher, /tf_static
├── left_wheel                       ← robot_state_publisher + /joint_states
├── right_wheel                      ← robot_state_publisher + /joint_states
└── caster                           ← robot_state_publisher, /tf_static
```

## Krok 1. Czujnik we własnym linku

Czujnik umieść w osobnym linku, przymocowanym do korpusu przegubem `fixed`. **Nazwa linku staje się nazwą ramki w TF.** Pozycję podaj względem rodzica (`relative_to`), wtedy liczby w TF są takie same jak w `models/vehicle/model.sdf`:

```xml
<link name='lidar_link'>
  <pose relative_to='chassis'>0 0 0.305 0 0 0</pose>
  <inertial>...</inertial>       <!-- wymagane, małe wartości -->
  <visual>...</visual>
  <sensor name='rplidar_c1' type='gpu_lidar'>
    <gz_frame_id>lidar_link</gz_frame_id>
    ...
  </sensor>
</link>

<joint name='lidar_joint' type='fixed'>
  <parent>chassis</parent>
  <child>lidar_link</child>
</joint>
```

Kamera wygląda tak samo: link `camera_link` z pozycją `1.02 0 0.2 0 0 0` względem `chassis`, przegub `camera_joint` i czujnik `rgbd_camera` z `<gz_frame_id>camera_link</gz_frame_id>`.

Orientację czujnika zadaje się trzema ostatnimi liczbami `<pose>` (roll, pitch, yaw). Przykładowo kamera pochylona o 15° w dół to `1.02 0 0.2 0 0.2618 0`. Dodatni pitch obraca oś x w dół.

## Krok 2. `<gz_frame_id>` musi być równe nazwie linku

Bez `<gz_frame_id>` Gazebo wpisuje do `frame_id` długą nazwę złożoną z modelu, linku i czujnika. Takiej ramki nie ma w drzewie TF, więc RViz nie wyświetli danych (*No transform from ...*).

Ostrzeżenie `XML Element[gz_frame_id] ... not defined in SDF` przy starcie można zignorować, bo element i tak jest odczytywany.

## Krok 3. Publikowanie TF z modelu: robot_state_publisher + sdformat_urdf

Nie przepisuj pozycji czujników ręcznie do `static_transform_publisher`. `robot_state_publisher` potrafi wczytać plik SDF, bo pakiet `sdformat_urdf` konwertuje go do URDF. Pozycje są wtedy zapisane w jednym miejscu, czyli w modelu.

```python
with open(os.path.join(pkg_simple_example, 'models', 'vehicle', 'model.sdf')) as f:
    robot_desc = f.read()

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': True}, {'robot_description': robot_desc}],
)
```

- Przeguby `fixed` (kamera, lidar) trafiają na `/tf_static`.
- Koła są przegubami ruchomymi i potrzebują `/joint_states`. W świecie działa plugin `JointStatePublisher`, a most przekazuje `/world/demo/model/vehicle/joint_state` jako `/joint_states`. Bez tego kamera i lidar nadal działają, ale w RViz nie pojawią się koła.
- Ostrzeżenia `URDF does not support <sensor>`, `KDL does not support a root link with an inertia` oraz `Converting unknown joint type ... caster_wheel into a fixed joint` są niegroźne.

Po zmianie pozycji czujnika w `model.sdf` wystarczy przebudować pakiet (`colcon build`). TF zmieni się sam.

## Krok 4. Czas symulacji

Wszystkie węzły, które publikują albo czytają TF (`robot_state_publisher`, RViz, Twoje węzły), uruchamiaj z `use_sim_time: True`. Most musi przekazywać `/clock`. Mieszanie czasu ściennego i symulacyjnego daje błędy typu *extrapolation into the future* / *past*.

## Ramka optyczna kamery

Czujnik `rgbd_camera` publikuje obraz RGB, głębię i chmurę punktów z jednym `frame_id`. Chmura punktów jest w konwencji linku (x do przodu), dlatego `gz_frame_id` ustawiono na `camera_link`. W RViz chmura wyświetla się wtedy poprawnie.

Narzędzia przetwarzające obrazy (`depth_image_proc`, `image_geometry`, kalibracja) zakładają jednak ramkę optyczną (z do przodu). Jeśli ich potrzebujesz:

1. Rozdziel kamerę na dwa czujniki w tym samym linku: `camera` (RGB) z `<gz_frame_id>camera_color_optical_frame</gz_frame_id>` oraz `depth_camera` / `rgbd_camera` z ramką `camera_link`.
2. Dodaj ramkę optyczną, czyli sam obrót bez przesunięcia:

   ```python
   Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       arguments=['--roll', '-1.5708', '--yaw', '-1.5708',
                  '--frame-id', 'camera_link',
                  '--child-frame-id', 'camera_color_optical_frame'],
       parameters=[{'use_sim_time': True}],
   )
   ```

Prawdziwy sterownik `realsense2_camera` publikuje takie ramki sam.

## Przejście na prawdziwy sprzęt

- **RealSense:** `realsense2_camera` publikuje własne ramki (`camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame`, ...). Od robota potrzebna jest tylko jedna transformacja `chassis → camera_link` (pozycja montażu).
- **RPLIDAR C1:** `rplidar_ros` publikuje skan z ramką ustawianą parametrem `frame_id` (domyślnie `laser`). Ustaw `frame_id:=lidar_link` albo dodaj transformację do `laser`. Sprawdź też, w którą stronę wskazuje 0° skanu: postaw przeszkodę przed robotem i zobacz, gdzie pojawia się w RViz. W razie potrzeby obróć ramkę lidaru o yaw π.

## Krok 5. Weryfikacja

Uruchom symulację:

```bash
ros2 launch simple_example vehicle_sensors.launch.py
```

Obraz całego drzewa (zapisuje `frames_<data>.pdf` w bieżącym katalogu):

```bash
ros2 run tf2_tools view_frames
```

Pozycje czujników względem korpusu:

```bash
ros2 run tf2_ros tf2_echo chassis lidar_link    # Translation: [0.000, 0.000, 0.305]
ros2 run tf2_ros tf2_echo chassis camera_link   # Translation: [1.020, 0.000, 0.200]
```

`frame_id` danych musi zgadzać się z nazwami w drzewie:

```bash
ros2 topic echo --once /scan --field header                               # frame_id: lidar_link
ros2 topic echo --once /camera/camera/depth/color/points --field header   # frame_id: camera_link
```

W RViz (*Fixed Frame*: `chassis`) włącz wyświetlacz **TF**. Skan lidaru i chmura punktów powinny pokrywać się z przeszkodami. Dobry test to wstawienie w Gazebo pudełka przed pojazdem: oba czujniki muszą pokazać je w tym samym miejscu. Przy *Fixed Frame* `chassis` robot stoi w RViz w miejscu, a podczas jazdy przesuwa się otoczenie.

## Typowe błędy

| Objaw | Przyczyna | Rozwiązanie |
|---|---|---|
| RViz: *No transform from [...] to [chassis]* | `frame_id` danych nie istnieje w drzewie | ustaw `<gz_frame_id>` równe nazwie linku (krok 2) |
| Brak ramek czujników w `view_frames` | nie działa `robot_state_publisher` albo model nie ma przegubu `fixed` | krok 1 i 3 |
| Skan i chmura punktów nie pokrywają się | zła pozycja albo orientacja w `<pose>` linku | popraw `model.sdf`, sprawdź `tf2_echo` |
| Chmura punktów obrócona o 90° | ramka optyczna użyta dla danych w konwencji linku (lub odwrotnie) | sekcja [Ramka optyczna kamery](#ramka-optyczna-kamery) |
| Skan lidaru odwrócony o 180° (prawdziwy lidar) | 0° skanu wskazuje inny kierunek niż w modelu | obróć ramkę lidaru o yaw π |
| *Lookup would require extrapolation into the future/past* | mieszanie czasu ściennego i symulacji | `use_sim_time: True` wszędzie, mostek `/clock` |
| Koła nie pojawiają się w RViz | brak `/joint_states` | plugin `JointStatePublisher` + wpis w moście |
