# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'driver.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(660, 553)
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        MainWindow.setAnimated(True)
        MainWindow.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(490, 290, 61, 21))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(30, 190, 113, 20))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.battery_voltage = QtWidgets.QLCDNumber(self.centralwidget)
        self.battery_voltage.setGeometry(QtCore.QRect(150, 190, 64, 23))
        self.battery_voltage.setObjectName("battery_voltage")
        self.battery_current = QtWidgets.QLCDNumber(self.centralwidget)
        self.battery_current.setGeometry(QtCore.QRect(150, 220, 64, 23))
        self.battery_current.setObjectName("battery_current")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_3.setGeometry(QtCore.QRect(30, 220, 113, 20))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.speed = QtWidgets.QProgressBar(self.centralwidget)
        self.speed.setGeometry(QtCore.QRect(260, 180, 131, 31))
        self.speed.setProperty("value", 24)
        self.speed.setOrientation(QtCore.Qt.Horizontal)
        self.speed.setObjectName("speed")
        self.lineEdit_6 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_6.setGeometry(QtCore.QRect(30, 130, 113, 20))
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.lineEdit_7 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_7.setGeometry(QtCore.QRect(30, 100, 113, 20))
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.motor_rpm = QtWidgets.QLCDNumber(self.centralwidget)
        self.motor_rpm.setGeometry(QtCore.QRect(150, 100, 64, 23))
        self.motor_rpm.setObjectName("motor_rpm")
        self.motor_temp = QtWidgets.QLCDNumber(self.centralwidget)
        self.motor_temp.setGeometry(QtCore.QRect(150, 130, 64, 23))
        self.motor_temp.setObjectName("motor_temp")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(260, 160, 54, 12))
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(470, 70, 91, 201))
        self.label_3.setText("")
        self.label_3.setPixmap(QtGui.QPixmap("carwheel.jpg"))
        self.label_3.setScaledContents(True)
        self.label_3.setObjectName("label_3")
        # lcdNumber 는 왼쪽바퀴 (보는사람 입장)
        self.lcdNumber = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber.setGeometry(QtCore.QRect(410, 100, 64, 23))
        self.lcdNumber.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.lcdNumber.setFrameShadow(QtWidgets.QFrame.Plain)
        self.lcdNumber.setSmallDecimalPoint(False)
        self.lcdNumber.setDigitCount(4)
        self.lcdNumber.setObjectName("lcdNumber")

        # lcdNumber
        # self.lcdNumber_2 = QtWidgets.QLCDNumber(self.centralwidget)
        # self.lcdNumber_2.setGeometry(QtCore.QRect(560, 100, 64, 23))
        # self.lcdNumber_2.setObjectName("lcdNumber_2")
        # self.lcdNumber_3 = QtWidgets.QLCDNumber(self.centralwidget)
        # self.lcdNumber_3.setGeometry(QtCore.QRect(410, 220, 64, 23))
        # self.lcdNumber_3.setObjectName("lcdNumber_3")
        self.lineEdit_8 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_8.setGeometry(QtCore.QRect(460, 50, 113, 20))
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.lcdNumber_4 = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber_4.setGeometry(QtCore.QRect(560, 220, 64, 23))
        self.lcdNumber_4.setObjectName("lcdNumber_4")
        # self.tableWidget = QtWidgets.QTableWidget(self.centralwidget)
        # self.tableWidget.setGeometry(QtCore.QRect(40, 280, 171, 191))
        # self.tableWidget.setObjectName("tableWidget")
        # self.tableWidget.setColumnCount(0)
        # self.tableWidget.setRowCount(0)
        self.throttle = QtWidgets.QProgressBar(self.centralwidget)
        self.throttle.setGeometry(QtCore.QRect(480, 310, 111, 23))
        self.throttle.setProperty("value", 24)
        self.throttle.setObjectName("throttle")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 660, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.lineEdit.setText(_translate("MainWindow", "<Throttle>"))
        self.lineEdit_2.setText(_translate("MainWindow", "Battery Voltage"))
        self.lineEdit_3.setText(_translate("MainWindow", "Battery Current"))
        self.speed.setFormat(_translate("MainWindow", "%p km/h"))
        self.lineEdit_6.setText(_translate("MainWindow", "Motor Temp."))
        self.lineEdit_7.setText(_translate("MainWindow", "Motor RPM"))
        self.label.setText(_translate("MainWindow", "Speed"))
        self.lineEdit_8.setText(_translate("MainWindow", "    <Wheel RPM>"))

if __name__=="__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    sys.exit(app.exec_())