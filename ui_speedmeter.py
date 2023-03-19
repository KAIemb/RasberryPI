# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'speedmeter.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from kaican_msg.msg import kacican_msg

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(477, 514)
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        MainWindow.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(40, 300, 51, 20))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(40, 330, 113, 20))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.battery_voltage = QtWidgets.QLCDNumber(self.centralwidget)
        self.battery_voltage.setGeometry(QtCore.QRect(160, 330, 64, 23))
        self.battery_voltage.setObjectName("battery_voltage")
        self.battery_current = QtWidgets.QLCDNumber(self.centralwidget)
        self.battery_current.setGeometry(QtCore.QRect(160, 360, 64, 23))
        self.battery_current.setObjectName("battery_current")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_3.setGeometry(QtCore.QRect(40, 360, 113, 20))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.lineEdit_4 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_4.setGeometry(QtCore.QRect(40, 390, 113, 20))
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.lineEdit_5 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_5.setGeometry(QtCore.QRect(40, 420, 113, 20))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar.setGeometry(QtCore.QRect(240, 331, 131, 20))
        self.progressBar.setProperty("value", 0)
        self.progressBar.setOrientation(QtCore.Qt.Horizontal)
        self.progressBar.setObjectName("progressBar")
        self.progressBar_2 = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar_2.setGeometry(QtCore.QRect(240, 360, 131, 16))
        self.progressBar_2.setProperty("value", 0)
        self.progressBar_2.setOrientation(QtCore.Qt.Horizontal)
        self.progressBar_2.setObjectName("progressBar_2")
        self.lineEdit_6 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_6.setGeometry(QtCore.QRect(40, 270, 113, 20))
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.lineEdit_7 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_7.setGeometry(QtCore.QRect(40, 240, 113, 20))
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.motor_rpm = QtWidgets.QLCDNumber(self.centralwidget)
        self.motor_rpm.setGeometry(QtCore.QRect(160, 240, 64, 23))
        self.motor_rpm.setObjectName("motor_rpm")
        self.motor_temp = QtWidgets.QLCDNumber(self.centralwidget)
        self.motor_temp.setGeometry(QtCore.QRect(160, 270, 64, 23))
        self.motor_temp.setObjectName("motor_temp")

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 477, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        # ROS 노드 초기화 
        rospy.init_node('qt_ros_node', anonymous=True)
        
        # ROS topic 구독자 생성
        '''
        topic_name: 구독할 topic의 이름입니다.
        topic_type: topic의 메시지 타입입니다. 이는 kaican_msg.msg에서 정의한 kacican_msg와 같은 메시지 타입 객체입니다.
        callback: topic에서 새로운 메시지가 수신될 때마다 호출되는 콜백 함수입니다. 이 콜백 함수는 메시지를 인자로 받습니다.
        '''
        rospy.Subscriber('battery_voltage', kacican_msg, self.update_battery_voltage)
        rospy.Subscriber('battery_current', kacican_msg, self.update_battery_current)
        rospy.Subscriber('motor_rpm', kacican_msg, self.update_motor_rpm)
        rospy.Subscriber('motor_temp', kacican_msg, self.update_motor_temp)
        rospy.Subscriber('progress', kacican_msg, self.update_progressBar)
        rospy.Subscriber('progress_2', kacican_msg, self.update_progressBar_2)

    def update_progressBar(self, data):
        # 들어온 데이터를 progressBar 범위로 매핑하여 값 설정
        mapped_value = int(data.data * self.progressBar.maximum())
        self.progressBar.setValue(mapped_value)

    def update_progressBar_2(self, data):
        # 들어온 데이터를 progressBar 범위로 매핑하여 값 설정
        mapped_value = int(data.data * self.progressBar_2.maximum())
        self.progressBar_2.setValue(mapped_value)

    # update battery voltage ui    
    def update_battery_voltage(self, data):
        self.battery_voltage.display(data.data)
        
    # battery_current topic update ui
    def update_battery_current(self, data):
        self.battery_current.display(data.data)

    # battery_current topic update ui
    def update_motor_rpm(self, data):
        self.motor_rpm.display(data.data)

    # motor_temp topic update ui
    def update_motor_temp(self, data):
        self.motor_temp.display(data.data)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.lineEdit.setText(_translate("MainWindow", "Throttle"))
        self.lineEdit_2.setText(_translate("MainWindow", "Battery Voltage"))
        self.lineEdit_3.setText(_translate("MainWindow", "Battery Current"))
        self.lineEdit_4.setText(_translate("MainWindow", "Lap time"))
        self.lineEdit_5.setText(_translate("MainWindow", "Lap "))
        self.progressBar.setFormat(_translate("MainWindow", "%p km/h"))
        self.progressBar_2.setFormat(_translate("MainWindow", "%p km/h"))
        self.lineEdit_6.setText(_translate("MainWindow", "Motor Temp."))
        self.lineEdit_7.setText(_translate("MainWindow", "Motor RPM"))


if __name__=="__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    sys.exit(app.exec_())