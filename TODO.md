### Za nardit:
##### OpenCV:
* Ureditev kode - trenutno vse v classu ImageConverter, naredi funkcije, class da bo bolj pregleden.

##### ROS Koordinatni sistemi:
##### RViz:
* Ureditev spawnanja in brisanja objektov - npr. kamera naredi sliko ko robot ne zakriva mize - spawna objekte. Ko objekt pobere in premakne na drugo lokacijo se robot zopet umakne, kamera slika in popravi lokacijo objektov (na prvi lokaciji zbriše, na drugi doda).

##### ROS Moveit!:
* Zaklenitev zadnjih dneh joint-ov robota (5 in 6), da bo gripper vedno pravokoten na mizo, klešče pa vzporedne z daljšim robom mize.
* Mehko približevanje in oddaljevanje od objektov - upočasnitev, z MoveIt Pick and place.
* Nastavitev pogojev, da pri pobiranju gripper stisne, pri odlaganju spusti.

##### ROS na TP:
* Naložitev interfaca, da lahko upravljam z Fanucovimi I/O - za premikanje gripperja.

### Narejeno:
##### Povezava, driverji:
* Kamera povezana z usb_cam driverjem - komanda za kamera node: 
```
source camera_ws/devel/setup.bash
rosrun usb_cam usb_cam_node
```
##### OpenCV: 
* Delujoča pretvorba iz ROS slik v OpenCV slike z uporabo cv_bridge, urejen CMakeLists in package.xml - referenca http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages (trenutno nariše krog na fiksnih koordinatah žive slike s kamere), za pogon opencv noda naslednji komandi: 
```
source camera_to_cv/devel/setup.bash
rosrun camera_to_cv camera_to_cv_node
``` 
* Zaznavanje posameznih valjev (center, radij) s thresholdingom + zaznavanje oblike kroga
* Zazanavanje mej mize s Canny edge detectorjem in shape detectorjem
* Zaznavanje lukenj (center, radij) z zaznavanjem oblike kroga

##### ROS Koordinatni sistemi:
* Transformacija med koordinatnim sistemom kamere in koordinatnim sistemom robota. Iz točke, ki jo vidi kamera v svojem k.s. se dobi točko v robotovem k.s. - za začetek je z-koordinata točk fiksna (zaradi uporabe ene kamere - sredina valja).

##### RViz:
* Iz koordinat detektiranih objektov in mize je narisana miza in objekti v Rviz - message za koordinate valjev je preko rostopic-a poslan v node basic_shapes, v tem node-u se izrišejo valji in pa miza, ki ima fiksne koordinate.

##### ROS Moveit!:
* Dobljen model gripperja, popravljen model "klešč", oba vključena v Moveit!
* Robot premaknjen v kartezičnem koordinatnem sistemu.

