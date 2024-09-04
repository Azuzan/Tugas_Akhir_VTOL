Link aplikasi mission planner: https://ardupilot.org/planner/docs/mission-planner-installation.html

1.	KONEKSI DENGAN PIXHAWK
  
     Koneksi dari computer ke Pixhawk bisa dengan USB dan Telemetri

![image](https://github.com/user-attachments/assets/8c54c6a0-2508-4d7d-9be7-d96eee10ad31)

USB

![image](https://github.com/user-attachments/assets/63cd0dbc-6e55-4df5-ad00-8beaa5457d30)

 
Telemetri


Pada Mission Planner, koneksi dan kecepatan data diatur pada bagian kanan atas layar.

![image](https://github.com/user-attachments/assets/41d41785-7344-4d6b-9f1b-d877995fda0e)
 
  Jika gambar sudah berubah maka koneksi berhasil.

![image](https://github.com/user-attachments/assets/94888e97-8c27-45e0-b2ca-82861c9526c3)
 
2.	KALIBRASI
3.	2.1	Accelerometer and Compass Calibration
![image](https://github.com/user-attachments/assets/882aeb2e-ac0c-4962-9bf4-1c136c70dddf)
![image](https://github.com/user-attachments/assets/75278ea8-4fcf-49fb-a020-365821a1c335)

   
  Buka "Initial Setup > Mandatory Hardware > Accel Calibration". Click Calibrate Accel to start the calibration. Arahkan hidung drone ke posisi LEVEL, LEFT, RIGHT, NOSEDOWN, NOSEUP, and BACK seperti perintah yang ada dibawahnya. "Calibration successful" akan tampil jika kalibrasi selesai. Untuk kalibrasi yang akurat, kendaraan atau perangkat harus diposisikan dalam enam orientasi berbeda:
  
  •	LEVEL	    : Posisi normal, datar, seperti ketika keadaan stabil di tanah.
  
  •	LEFT	    : Miring ke kiri.
  
  •	RIGHT	    : Miring ke kanan.
  
  •	NOSEDOWN	: Hidung kendaraan menghadap ke bawah.
  
  •	NOSEUP	  : Hidung kendaraan menghadap ke atas.
  
  •	BACK	    : hampir sama seperti level namun posisi drone dibalik.
  
  
  2.2	Compass Calibration
  
  Buka "Initial Setup > Mandatory Hardware > Compass".
  
  ![image](https://github.com/user-attachments/assets/b4b73c84-f556-4fc6-a5ad-c912fc3524fc)
   
  
  Tampilan compass priority pada mission planer. Pilih start untuk mulai kalibrasi. 
  
  Saat proses kalibrasi angkat dan putar-putar drone sampai proses selesai
  Ikuti proses seperti https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html#compass-calibration
  
  Setelah proses selesai Reboot perlu dilakukan.
  Tekan ctrl+f pada mission planner pilih reboot Pixhawk.

  ![image](https://github.com/user-attachments/assets/4f4af757-807e-451f-8c4a-98158998a8d0)

 
3. Planning a Mission with Waypoints

  Setelah terkoneksi klik kanan untuk set EKF Origin here untuk set posisi home.

  ![image](https://github.com/user-attachments/assets/31e7bf44-a8ec-4855-a595-be69bfed7796)

   
  Setelah home di set buka menu “PLAN”.

  ![image](https://github.com/user-attachments/assets/61b0f437-f0dd-4681-bffb-be7f2f546eb3)

   
  Alur perjalanan drone dapat dibuat dengan menambah list misi yang akan dilakukan. Pastikan delay dan ketinggian(Alt) di set dengan benar. Setelah Planning selesai write untuk memasukan program.
  MATIKAN SAFETY SWITCH untuk dapat menyalakan motor.

  ![image](https://github.com/user-attachments/assets/2229f8a6-5617-40c9-a697-8c5371e6cfd7)

   
  Untuk masuk ke mode auto pastikan switch berada pada posisi Stabilize dan Stabilize. Arahkan joystick kiri ke kanan bawah untuk memutar motor, setelah motor menyala naikan posisi throttle sedikit agar kecepatan motor bertambah sedikit, pindahkan switch Stabilize ke AUTO untuk masuk ke program yang telah dibuat. Drone akan masuk ke list pertama yaitu Takeoff dan akan lanjut ke urutan berikutnya.
  Drone dapat dipaksa mendarat saat penerbangan Auto dengan merubah switch dari Stabilize ke LAND.
