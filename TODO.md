### Za naredit:
##### OpenCV:
* Zamenjava Publisher/Subscriber s Client/Server -> camera node steče samo takrat ko ga pokličemo - ne išče stalno lukenj in objektov.

##### ROS Moveit!:
* Uporaba Pick and place interface - ko je objekt pobran naredi, da je v vizualizaciji objekt stisnjen v gripperju

##### RViz:
* Optimizacija programa, ki spawna objekte

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
rosrun camera_to_cv camera_to_cv_node
``` 
* Zaznavanje posameznih valjev (center, radij) s thresholdingom + zaznavanje oblike kroga
* Zazanavanje mej mize s Canny edge detectorjem in shape detectorjem
* Zaznavanje lukenj (center, radij) z zaznavanjem oblike kroga

##### ROS Koordinatni sistemi:
* Transformacija med koordinatnim sistemom kamere in koordinatnim sistemom robota. Iz točke, ki jo vidi kamera v svojem k.s. se dobi točko v robotovem k.s. - za začetek je z-koordinata točk fiksna (zaradi uporabe ene kamere - sredina valja).

##### RViz:
* Iz koordinat detektiranih objektov in mize je narisana miza in objekti v Rviz - message za koordinate valjev je preko rostopic-a poslan v node basic_shapes, v tem node-u se izrišejo valji in pa miza, ki ima fiksne koordinate.
* Ureditev spawnanja in brisanja objektov - kamera naredi sliko ko robot ne zakriva mize - spawna objekte. Lokacije objektov so zopet popravljene ko kamera spet vidi mizo.
* Rviz vizualizacija se požene z naslednjo komando
```
rosrun rviz_markers basic_shapes
``` 

##### ROS Moveit!:
* Dobljen model gripperja, popravljen model "klešč", oba vključena v Moveit!
* Robot premaknjen v kartezičnem koordinatnem sistemu.
* Zaklenitev zadnjih dneh joint-ov robota (5 in 6), da je gripper vedno pravokoten na mizo, klešče pa vzporedne z daljšim robom mize.
* Mehko približevanje in oddaljevanje od objektov
* Nastavitev pogojev, da pri pobiranju gripper stisne, pri odlaganju spusti.
* Programiranje rutine pobiranja in odlaganja - npr. pobere vse objekte in jih odloži na min x in min y koordinate luknje, ki je še prosta.
* Moveit! rutina se zažene z naslednjo komando, v RViz-u pa je potrebno dodati še RvizVisualToolsGui. Pred tem se prepričaj, da je pognan tudi server za upravljanje gripperja (set_io).
```
rosrun moveit_fanuc move_group_interface_tutorial
```
* Naprogramirane ovire (ohišje robota)

##### URDF:
* Narejen URDF file za Fanuc LR Mate 200 iD 4s
* Požene se s komando
```
roslaunch fanuc_lrmate200id_moveit_config moveit_planning_execution.launch 
```

##### ROS na TP:
* Naložitev driverja fanuc_cgio, da lahko upravljam z Fanucovimi I/O - za odpiranje/zapiranje gripperja s pomočjo GET komand.

##### Razno:
* Narejen server/client povezava za upravljanje gripperja. Server za uporabo gripperja se zažene z naslednjo komando:
```
rosrun set_io set_io_server
```

### Delo za naprej / ko bo narejeno vse za pobiranje z eno kamero in enakimi barvami valjev:
##### Različne barve valjev:
* zaznavanje barve valjev in odlaganje v škatlo npr. rdeči, zeleni, modri, beli
* vizualizacija različnih barv v RViz
##### Interakcija:
* Interaktivni vmesnik za nastavljanje kam s pobranimi objekti
* Označi objekte, prepoznaj vzorce in zaznavaj te objekte (fiksne višine)
##### Stereo kamera:
* npr. dodajanje Kinect senzorja, ki nam omogoči pobiranje in odlaganje različnih predmetov
* zaznavanje različnih objektov s Kinectom - za objekt različna rutina kako ga gripper prime
* vizualizacija različnih objektov v RViz


