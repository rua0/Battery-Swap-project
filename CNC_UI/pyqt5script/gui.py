# import sys
# from PyQt5 import QtGui
# from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton


# class Window(QMainWindow):

# 	def __init__(self):
# 		super(Window, self).__init__()
# 		self.setGeometry(50,50,500,300)
# 		self.setWindowTitle("my GUI")
# 		self.show()

# 	def home(self):
# 		btn = QPushButton("Quit", self)
# 		#btn.clicked.connect(QtCore.QCoreApplication.instance().quit)
# 		btn.clicked.connect(self.close_application)

# 		btn.resize(100,100)
# 		btn.move(100,100)

# 	def close_application(self):
# 		print("I got works done")
# 		sys.exit()


# app = QApplication(sys.argv)
# GUI = Window()
# sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
# from PyQt5.QtGui import QIcon
# from PyQt5.QtCore import pyqtSlot
 
# class App(QWidget):
 
#     def __init__(self):
#         super().__init__()
#         self.title = 'PyQt5 button - pythonspot.com'
#         self.left = 10
#         self.top = 10
#         self.width = 320
#         self.height = 200
#         self.initUI()
 
#     def initUI(self):
#         self.setWindowTitle(self.title)
#         self.setGeometry(self.left, self.top, self.width, self.height)
 
#         button = QPushButton('PyQt5 button', self)
#         button.setToolTip('This is an example button')
#         button.move(100,70) 
#         button.clicked.connect(self.on_click)
 
#         self.show()
 
#     @pyqtSlot()
#     def on_click(self):
#         print('PyQt5 button click')
 
# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     ex = App()
#     sys.exit(app.exec_())


import sys
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QAction, QTabWidget,QVBoxLayout, QLineEdit, QMessageBox
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot

from PyQt5 import QtCore, QtGui
from PyQt5 import QtWidgets


class DragButton(QtWidgets.QLabel):

    def mousePressEvent(self, event):
        self.__mousePressPos = None
        self.__mouseMovePos = None
        if event.button() == QtCore.Qt.LeftButton:
            self.__mousePressPos = event.globalPos()
            self.__mouseMovePos = event.globalPos()

        super(DragButton, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            # adjust offset from clicked point to origin of widget
            currPos = self.mapToGlobal(self.pos())
            globalPos = event.globalPos()
            diff = globalPos - self.__mouseMovePos

            print (diff)


            newPos = self.mapFromGlobal(currPos + diff)
            # print("Qpoint")
            # print(newPos.x())
            # print(newPos.y())

            #print (type(newPos))
            dis = (newPos.x()-175)**2 + (newPos.y()-165)**2
            if dis<=5625:
                self.move(newPos)
                self.__mouseMovePos = globalPos
            #else:
                #make the interpolation here

            # self.move(newPos)

            # self.__mouseMovePos = globalPos

        super(DragButton, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.__mousePressPos is not None:
            moved = event.globalPos() - self.__mousePressPos 
            if moved.manhattanLength() > 3:
                event.ignore()
                return

        super(DragButton, self).mouseReleaseEvent(event)




 
class App(QMainWindow):
 
    def __init__(self):
        super().__init__()
        self.title = 'PyQt5 tabs - pythonspot.com'
        # self.left = 0
        # self.top = 0
        # self.width = 800
        # self.height = 600
        self.resize(800,600)
        self.setWindowTitle(self.title)
        #self.setGeometry(self.left, self.top, self.width, self.height)
 
        self.table_widget = MyTableWidget(self)
        self.setCentralWidget(self.table_widget)
 
        self.show()
 
class MyTableWidget(QWidget):        
 
    def __init__(self, parent):   
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout(self)
 
        # Initialize tab screen
        self.tabs = QTabWidget()
        self.tab1 = QWidget()	
        self.tab2 = QWidget()
        self.tab3 = QWidget()
        self.tabs.resize(800,600) 
 
        # Add tabs
        self.tabs.addTab(self.tab1,"Tab 1")
        self.tabs.addTab(self.tab2,"Tab 2")
        self.tabs.addTab(self.tab3,"Tab 3")
 
        # Create first tab
        self.tab1.layout = QVBoxLayout(self)
        self.pushButton1 = QPushButton("PyQt5 button")
        self.pushButton1.clicked.connect(self.on_click)
        self.tab1.layout.addWidget(self.pushButton1)
        self.tab1.setLayout(self.tab1.layout)

        #Create second tab
        # self.tab2.layout = QVBoxLayout(self)
        # self.pushButton2 = QPushButton("PyQt5 button")
        # self.pushButton2.clicked.connect(self.on_click)
        # self.tab2.layout.addWidget(self.pushButton2)
        # self.tab2.setLayout(self.tab2.layout)


        self.tab2.layout = QVBoxLayout(self)
        # Create textbox
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 20)
        self.textbox.resize(280,40)

 
        # Create a button in the window
        self.button = QPushButton('Show text', self)
        self.button.move(20,80)
 
        # connect button to function on_click
        self.button.clicked.connect(self.on_click2)
        
        self.tab2.layout.addWidget(self.textbox)
        self.tab2.layout.addWidget(self.button)
        self.tab2.setLayout(self.tab2.layout)


        self.tab3.layout = QVBoxLayout(self)
        self.l1 = QtWidgets.QLabel(self)
        self.l1.setPixmap(QtGui.QPixmap('bg.png'))
        self.l1.move(200,300)
        
        self.lbutton = DragButton(self)
        self.lbutton.setPixmap(QtGui.QPixmap('thumb.png'))
        self.lbutton.move(100+75,90+75)

        self.tab3.layout.addWidget(self.l1)
        self.tab3.layout.addWidget(self.lbutton)
        self.tab3.setLayout(self.tab3.layout)




 
        # Add tabs to widget        
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)
 
    @pyqtSlot()
    def on_click(self):
        print("this is the output of button")

    def on_click2(self):
        textboxValue = self.textbox.text()
        QMessageBox.question(self, 'Message - pythonspot.com', "You typed: " + textboxValue, QMessageBox.Ok, QMessageBox.Ok)
        self.textbox.setText("")
 


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())

    


