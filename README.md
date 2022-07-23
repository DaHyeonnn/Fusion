# Fusion
## - Camera LiDAR Sensor : late Fusion

카메라와 라이다의 좌표계가 달라 센서 퓨전을 위한 캘리브레이션을 시도해야한다.
![image](https://user-images.githubusercontent.com/90945094/180591958-2fe871fe-09d8-4bbc-93bc-76004526dcf6.png)

카메라의 내부행렬과 외부행렬(RT)을 구한 뒤, 
![image](https://user-images.githubusercontent.com/90945094/180591929-57ff9a31-c094-4cb2-8ab5-ab50e7223c4e.png)

아래의 공식에 대입해주면 된다.
![image](https://user-images.githubusercontent.com/90945094/180591950-9fefa5f2-e0ec-4e88-9c6a-5c72c3e9e0c2.png)

## 결과

![image](https://user-images.githubusercontent.com/90945094/180591962-89466a25-8fd7-415d-b851-ca1a5b85b529.png)
