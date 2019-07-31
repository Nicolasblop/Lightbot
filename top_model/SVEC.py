#!/usr/bin/python
'''
    Simulation Visualiser for Embedded Cadmium

    By: Ben Earle
    ARSLab - Carleton University

    This script will parse the I/O files and animate the pin values
    for the duration of the simulation. 

    Note, if tkinter is not installed by default run the following
    command in the terminal:
        sudo apt-get install python-tk
'''
from tkinter import filedialog
from tkinter import *
import tkMessageBox
import os

debug = True
#Constants to change units of time to micro seconds:
HOURS_TO_MICRO = 1000*1000*60*60
MIN_TO_MICRO   = 1000*1000*60
SEC_TO_MICRO   = 1000*1000
MILI_TO_MICRO  = 1000

def strTimeToMicroSeconds(time):
    intList = map(int, time.split(':'))
    while(len(intList) < 5):
        intList.append(0)
    return (intList[0] * HOURS_TO_MICRO + intList[1] * MIN_TO_MICRO + intList[2] * SEC_TO_MICRO + intList[3] * MILI_TO_MICRO + intList[4])

def microSecondsToStrTime(usec):
    hours = usec / HOURS_TO_MICRO
    usec = usec % HOURS_TO_MICRO

    minu = usec / MIN_TO_MICRO
    usec = usec % MIN_TO_MICRO

    sec = usec / SEC_TO_MICRO
    usec = usec % SEC_TO_MICRO

    msec = usec / MILI_TO_MICRO
    usec = usec % MILI_TO_MICRO

    return (str(hours).zfill(2) + ":" + str(minu).zfill(2) + ":" + str(sec).zfill(2) + ":" + str(msec).zfill(3) + ":" + str(usec).zfill(3))

# Helper function to read and return the contents of the file.
def loadFromDir(path):
    output = []
    # Read every file in directory
    for filename in os.listdir(path):
        events = []
        with open(path+filename, "r") as f:
            # Read each line of the file
            for line in f.readlines():
                events.append([strTimeToMicroSeconds(line.split(" ")[0]), line.split(" ")[1].strip("\n")])
        output.append([filename, events])
    return output

# Here, we are creating our class, Window, and inheriting from the Frame
# class. Frame is a class from the tkinter module. (see Lib/tkinter/__init__)
class Window(Frame):

    # Define settings upon initialization. Here you can specify
    def __init__(self, master=None):
        
        # Inlitialize class variables
        self.inputFolderPath  = ""
        self.outputFolderPath = ""
        self.loaded = False
        self.displayTime = 0
        self.stepList = []
        self.labelList = []

        # parameters that you want to send through the Frame class. 
        Frame.__init__(self, master)   

        #reference to the master widget, which is the tk window                 
        self.master = master

        #with that, we want to then run init_window, which doesn't yet exist
        self.init_window()

    #Creation of init_window
    def init_window(self):

        # changing the title of our master widget      
        self.master.title("SVEC")

        # allowing the widget to take the full space of the root window
        self.pack(fill=BOTH, expand=1)

        ################################################################
        # creating the menu 
        menu = Menu(self.master)
        self.master.config(menu=menu)

        # create the file object
        file = Menu(menu)

        # adds a command to the menu option, calling it exit, and the
        # command it runs on event is client_exit
        file.add_command(label="Open Input Folder", command=self.getInputFolder)
        file.add_command(label="Open Output Folder", command=self.getOutputFolder)
        file.add_command(label="Open Top Folder", command=self.getTopFolder)
        file.add_command(label="Exit", command=self.client_exit)
        #added "file" to our menu
        menu.add_cascade(label="File", menu=file)

        # # create the file object)
        # edit = Menu(menu)

        # # adds a command to the menu option, calling it exit, and the
        # # command it runs on event is client_exit
        # edit.add_command(label="Show Text", command=self.showText)

        # #added "file" to our menu
        # menu.add_cascade(label="Edit", menu=edit)
        ################################################################
        # Entry boxes:
        self.stepSize = Entry(self,width=14)
        self.stepSize.insert(0, "00:00:00:000:000")
        #self.stepSize.place(x=190, y=5)
        self.stepSize.grid(row = 2, column = 1)

        self.displayTimeEntry = Entry(self,width=14)
        self.displayTimeEntry.insert(1, "00:00:00:000:000")
        self.displayTimeEntry.grid(row = 1, column = 1)
        ################################################################
        # Make the buttons and place them in the grid.
        loadButton = Button(self, text="Reload",command=self.loadFiles)
        loadButton.grid(row = 0, column = 0)

        quitButton = Button(self, text="Exit",command=self.client_exit)
        quitButton.grid(row = 0, column = 3)

        revButton = Button(self, text="<<<",command=self.revTime)
        revButton.grid(row = 2, column = 2)

        fwdButton = Button(self, text=">>>",command=self.fwdTime)
        fwdButton.grid(row = 2, column = 3)

        revStepButton = Button(self, text=" |< ",command=self.revStepTime)
        revStepButton.grid(row =1, column = 2)

        fwdStepButton = Button(self, text=" >| ",command=self.fwdStepTime)
        fwdStepButton.grid(row = 1, column = 3)

        resetButton = Button(self, text="Reset Time",command=self.resetTime)
        resetButton.grid(row = 0, column = 1)

        setButton = Button(self, text="Set Time",command=self.setTime)
        setButton.grid(row = 0, column = 2)
        ################################################################
        # Text boxes
        stepLabel = Label(self, text="Step size:")
        stepLabel.grid(row = 2, column = 0)

        timeLabel = Label(self, text="Current time:")
        timeLabel.grid(row = 1, column = 0)

        inLabel = Label(self, text="Inputs")
        inLabel.grid(row = 3, column = 0)

        outLabel = Label(self, text="Outputs")
        outLabel.grid(row = 3, column = 2)


        ################################################################

    def showText(self):
        text = Label(self, text="Hello World!")
        text.pack()
        
    def updatePinDisplay(self):
        #If the files have not been loaded throw an error.
        if (not self.loaded):
            tkMessageBox.showinfo("ERROR", "Please load the I/O folders and try again.")
            return

        # Update the entry boxes:
        size = strTimeToMicroSeconds(self.stepSize.get())
        self.stepSize.delete(0, END)
        self.stepSize.insert(1, microSecondsToStrTime(size))
        
        self.displayTimeEntry.delete(0, END)
        self.displayTimeEntry.insert(1, microSecondsToStrTime(self.displayTime))

        for label in self.labelList:
            label.destroy()
        self.labelList = []

        i = 0
        for pin in self.inputPins:
            currEvent = []
            i = 1+i
            label = Label(self, text=pin[0].split('.')[0].strip("_In"))
            label.grid(row = 3+i, column = 0)

            for event in pin[1]:
                if (event[0] < self.displayTime):
                    currEvent = event
    
            if (currEvent == []):
                currEvent = [0,'?']
            label = Label(self, text=currEvent[1])
            label.grid(row = 3+i, column = 1)
            self.labelList.append(label)

        i=0
        for pin in self.outputPins:
            currEvent = []
            i = 1+i
            label = Label(self, text=pin[0].split('.')[0].strip("_Out"))
            label.grid(row = 3+i, column = 2)

            for event in pin[1]:
                if (event[0] < self.displayTime):
                    currEvent = event
            if (currEvent == []):
                currEvent = [0,'?']

            label = Label(self, text=currEvent[1])
            label.grid(row = 3+i, column = 3)
            self.labelList.append(label)

    def updateStepTimes(self):
        self.stepList = []
        for pin in self.inputPins:
            for event in pin[1]:
                self.stepList.append(event[0])
        
        for pin in self.outputPins:
            for event in pin[1]:
                self.stepList.append(event[0])
        self.stepList.sort()


    def loadFiles(self):
        if (self.inputFolderPath == "" or self.outputFolderPath == ""):
            self.loaded = False
            tkMessageBox.showinfo("ERROR", "Please load the I/O folders and try again.")
        else:
            self.loaded = True
            self.inputPins = loadFromDir(self.inputFolderPath)
            self.outputPins = loadFromDir(self.outputFolderPath)
            self.updateStepTimes();

    def revTime(self):
        self.displayTime -= strTimeToMicroSeconds(self.stepSize.get())
        if(self.displayTime < 0):
            self.displayTime = 0;
        self.updatePinDisplay()
        
    def fwdTime(self):
        self.displayTime += strTimeToMicroSeconds(self.stepSize.get())
        self.updatePinDisplay()


    def revStepTime(self):
        #If the files have not been loaded throw an error.
        if (not self.loaded):
            tkMessageBox.showinfo("ERROR", "Please load the I/O folders and try again.")
            return
        newTime = 0
        for time in self.stepList:
            if(time < self.displayTime):
                newTime = time

        self.displayTime = newTime
        self.updatePinDisplay()
        
    def fwdStepTime(self):
        #If the files have not been loaded throw an error.
        if (not self.loaded):
            tkMessageBox.showinfo("ERROR", "Please load the I/O folders and try again.")
            return
        for time in self.stepList:
            if(time > self.displayTime):
                self.displayTime = time
                self.updatePinDisplay()
                return
        self.displayTime = self.stepList[-1]
        self.updatePinDisplay()



    def resetTime(self):
        self.displayTime = 0
        self.updatePinDisplay()

    def setTime(self):
        self.displayTime = strTimeToMicroSeconds(self.displayTimeEntry.get())
        self.updatePinDisplay()

    def client_exit(self):
        exit()

    def getInputFolder(self):
        self.inputFolderPath = filedialog.askdirectory()
        if (not os.path.isdir(self.inputFolderPath)):
            self.loaded = False
            tkMessageBox.showinfo("ERROR", "Folders not found. Please try again.")
            self.inputFolderPath = ""

    def getOutputFolder(self):
        self.outputFolderPath = filedialog.askdirectory()
        if (not os.path.isdir(self.outputFolderPath)):
            self.loaded = False
            tkMessageBox.showinfo("ERROR", "Folders not found. Please try again.")
            self.outputFolderPath = ""

    def getTopFolder(self):
        top = filedialog.askdirectory()
        self.inputFolderPath = top + "/inputs/"
        self.outputFolderPath = top +"/outputs/"
        if ((not os.path.isdir(self.inputFolderPath)) or (not os.path.isdir(self.outputFolderPath))):
            tkMessageBox.showinfo("ERROR", "Folders not found. Try loading them indiviually.")
            self.inputFolderPath = ""
            self.outputFolderPath = ""
            self.loaded = False
        else:
            self.loadFiles()
            self.updatePinDisplay()




# root window created. Here, that would be the only window, but
# you can later have windows within windows.
root = Tk()

root.geometry("500x400")

#creation of an instance
app = Window(root)


#mainloop 
root.mainloop()  