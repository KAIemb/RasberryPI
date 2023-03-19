# RasberryPI
### Team kai 레포

2023/03/01 할일</br>
```
1. 라즈베리파이OS LEGACY (데비안10) os 설치
2. 라즈베리파이 ROS melodic 설치
3. 라즈베리파이 catkin_ws 환경 구축
4. 라즈베리파이 arduino ros 설치
5. 라즈베리파이에 아두이노우노 연결해서 아두이노우노 ROS 통신 노드 띄우기
6. CAN 통신 -> throttle 신호 받아서 투명 디스플레이 바를 이용해 표시하기
```
라즈베리파이 데비안10</br>
```
https://www.raspberrypi.org/downloads/raspbian/
```

### kaikan message package

아두이노 ros serial을 위한 custom message package </br>


```
/
├── catkin_ws/
    └── src
```

catkin_ws/src 폴더에 넣기
그 다음으로
```
catkin_make
```
실행</br>

새로운 브랜치에 넣고싶으면 아래 강제로 입력해서 브랜치로 넣으면됨
```
git push -f origin local-branch:remote-branch
```

ui_dirver.py 코드가 실질적으로 최종 update된 코드이다.
