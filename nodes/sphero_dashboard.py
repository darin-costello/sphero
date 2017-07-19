#!/usr/bin/python

import sys, rospy, math
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool

class LEDWidget(QtGui.QLabel):
   
    def __init__(self, size, rgb):
        super(QtGui.QLabel, self).__init__()
        pixmap = QtGui.QPixmap(size[0], size[1])
        self.setPixmap(pixmap)
        self.rgb = rgb
        self.setAutoFillBackground(True)
        self.setBackgroundRole(QtGui.QPalette.Base)

    def setRGB(self, r, g, b):
        self.rgb = [r,g,b]
        p = QtGui.QPalette()
        p.setColor(self.backgroundRole(), QtGui.QColor(self.rgb[0], self.rgb[1], self.rgb[2]))
        self.setPalette(p)
        self.update()
  
    def paintEvent(self, e):
        super(QtGui.QLabel, self).paintEvent(e)
        #qp = QtGui.QPainter()
        #qp.begin(self)
        #qp.end()         

class LEDConfig(QtGui.QWidget):

    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow.parentWindow
        self.initUI()
        self.updateConfig()
    
    def initUI(self):   

        self.bk_label = QtGui.QLabel("back LED")
        self.bk_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.bk_sl.setMinimum(0)
        self.bk_sl.setMaximum(255)
        self.bk_sl.setValue(self.parentWindow.backLEDVal)
        self.bk_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.bk_sl.setTickInterval(1)

        self.r_label = QtGui.QLabel("R")
        self.r_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.r_sl.setMinimum(0)
        self.r_sl.setMaximum(255)
        self.r_sl.setValue(self.parentWindow.ledRVal)
        self.r_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.r_sl.setTickInterval(1)
        self.r_sl.valueChanged.connect(self.valuechange)

        self.g_label = QtGui.QLabel("G")
        self.g_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.g_sl.setMinimum(0)
        self.g_sl.setMaximum(255)
        self.g_sl.setValue(self.parentWindow.ledGVal)
        self.g_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.g_sl.setTickInterval(1)
        self.g_sl.valueChanged.connect(self.valuechange)

        self.b_label = QtGui.QLabel("B")
        self.b_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.b_sl.setMinimum(0)
        self.b_sl.setMaximum(255)
        self.b_sl.setValue(self.parentWindow.ledBVal)
        self.b_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.b_sl.setTickInterval(1)
        self.b_sl.valueChanged.connect(self.valuechange)

        self.led = LEDWidget([20,20],
                             [self.parentWindow.ledRVal,
                              self.parentWindow.ledGVal,
                              self.parentWindow.ledBVal])
        
        self.btnUpdate = QtGui.QPushButton("Update")
        self.btnUpdate.clicked.connect(self.updateConfig)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.led)
        r_layout = QtGui.QHBoxLayout()
        r_layout.addWidget(self.r_label)
        r_layout.addWidget(self.r_sl)
        layout.addLayout(r_layout)
        g_layout = QtGui.QHBoxLayout()
        g_layout.addWidget(self.g_label)
        g_layout.addWidget(self.g_sl)
        layout.addLayout(g_layout)
        b_layout = QtGui.QHBoxLayout()
        b_layout.addWidget(self.b_label)
        b_layout.addWidget(self.b_sl)
        layout.addLayout(b_layout)
        bk_layout = QtGui.QHBoxLayout()
        bk_layout.addWidget(self.bk_label)
        bk_layout.addWidget(self.bk_sl)
        layout.addLayout(bk_layout)
        btn_layout = QtGui.QHBoxLayout()
        btn_layout.addWidget(self.btnUpdate)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        
        self.show()

    def updateConfig(self):
        
        self.parentWindow.ledRVal = self.r_sl.value()
        self.parentWindow.ledGVal = self.g_sl.value()
        self.parentWindow.ledBVal = self.b_sl.value()        
        self.parentWindow.setLEDColor(self.parentWindow.ledRVal,
                                 self.parentWindow.ledGVal,
                                 self.parentWindow.ledBVal)
        self.led.setRGB(self.parentWindow.ledRVal,
                        self.parentWindow.ledGVal,
                        self.parentWindow.ledBVal)

        self.parentWindow.backLEDVal = self.bk_sl.value()
        self.parentWindow.setBackLED(self.parentWindow.backLEDVal)
        self.update()

    def valuechange(self):
        self.led.setRGB(self.r_sl.value(),
                        self.g_sl.value(),
                        self.b_sl.value())
        self.led.update()

class DashboardWidget(QtGui.QWidget):

    def __init__(self, parent):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parent
        self.initUI()

    def initUI(self):
        
        self.ledConfig = LEDConfig(self) 
        self.stabilizationRadioButton = QtGui.QRadioButton("Disable Stabilization")
        self.stabilizationRadioButton.setChecked(False)
        self.stabilizationRadioButton.toggled.connect(self.handleStabilizationCheck)
 
        self.degLabel = QtGui.QLabel("Degree:")
        self.degTextbox = QtGui.QLineEdit()
        self.degTextbox.setText("15")
        self.degTextbox.setFixedWidth(40)
        self.leftBtn = QtGui.QPushButton("Counter Clockwise")
        self.leftBtn.clicked.connect(self.leftRotate)
        self.rightBtn = QtGui.QPushButton("Clockwise")
        self.rightBtn.clicked.connect(self.rightRotate)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.ledConfig)
        layout.addWidget(self.stabilizationRadioButton)
        ctrlLayout = QtGui.QHBoxLayout()
        ctrlLayout.addWidget(self.degLabel)
        ctrlLayout.addWidget(self.degTextbox)
        ctrlLayout.addWidget(self.leftBtn)
        ctrlLayout.addWidget(self.rightBtn)
        layout.addLayout(ctrlLayout)
        self.setLayout(layout)
        self.show()

    def updateHeading(self):
        val = self.headingSlider.value()
        self.parentWindow.setHeading(val)
        #self.headingSlider.setValue(180)
        self.update()

    def leftRotate(self):
        deg = int(self.degTextbox.displayText())
        self.parentWindow.setHeading(360-deg)

    def rightRotate(self):
        deg = int(self.degTextbox.displayText())
        self.parentWindow.setHeading(15)

    def headingChange(self, int):
        delta_val = self.headingSlider.value() - self.currentHeadingSliderValue 
        #self.parentWindow.setHeading(delta_val)
        self.currentHeadingSliderValue = self.headingSlider.value()



    def updateStablization(self, on):
        self.parentWindow.setStabilization(on)

    def handleStabilizationCheck(self):
        if self.stabilizationRadioButton.isChecked() == True:
            self.updateStablization(True)
        else:
            self.updateStablization(False)

class SpheroDashboardForm(QtGui.QMainWindow):
    
    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.resize(600, 300) 
        

        rospy.init_node('sphero_dashboard', anonymous=True)
        self.cmdTurnPub = rospy.Publisher('cmd_turn', Float32, queue_size=1)     
 
        self.ledPub = rospy.Publisher('set_color', ColorRGBA, queue_size=1)
        self.backLedPub = rospy.Publisher('set_back_led', Float32, queue_size=1)   
        self.headingPub = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.stabilizationPub = rospy.Publisher('disable_stabilization', Bool, queue_size=1)

        self.ledRVal = 0
        self.ledGVal = 0
        self.ledBVal = 0
        self.backLEDVal = 0
        
        #self.ledConfig.move(10, 10)
        self.setLEDColor(self.ledRVal, self.ledGVal, self.ledBVal)
        self.setBackLED(self.backLEDVal)

        self.initUI()


    def initUI(self):       
        self.dashboard = DashboardWidget(self)
        self.setCentralWidget(self.dashboard)
        self.setWindowTitle("Sphero dashboard")
        self.show() 

    def setLEDColor(self, r, g, b):
        color = ColorRGBA(float(r)/255.0, float(g)/255.0, float(b)/255.0, 1.0)
        self.ledPub.publish(color)

    def setBackLED(self, val):
        light = Float32()
        light.data = float(val)
        self.backLedPub.publish(light)

    def setHeading(self, val):
        turning = Float32()
        turning.data = float(val)
        self.cmdTurnPub.publish(turning)

        heading = Float32()
        heading.data = 0.0
        self.headingPub.publish(heading)

    def setStabilization(self, on):
        stab_data = Bool()
        stab_data.data = on
        self.stabilizationPub.publish(stab_data)        


        

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroDashboardForm()
    w.show()
    sys.exit(app.exec_())
  
        
