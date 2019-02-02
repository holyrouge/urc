from tkinter import *
import tkinter as tk
import tkinter.ttk as ttk
import tkinter.ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib as matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib import style
from threading import Thread
import pandas as pd
from pandas import Series
import math
import datetime
import csv
import time



f = Figure(figsize=(5, 5),dpi=100)  # adding figure f, which is of dimension 5x5 inches (I think) and has dpi of 100

# adding a default black background to all matplotlib graphs (may be overwritten depending on the particular graph)
matplotlib.style.use('dark_background')

img = plt.imread("hanksville_edit.png")

f, ax = plt.subplots()  # adding a figure 'f' and adding a subplot 'ax' to it
f.add_subplot(111, frameon=False)  # on figure f, adding subplot 1 of the 1x1 set of graphs (required syntax)

f2 = plt.Figure(figsize=(7, 7), dpi=100)  # for use in the graphs page
# The following subplots ax2, ax3, ax4, ax5 are all being put on a separate graphs page
ax2 = f2.add_subplot(221)  # adding graph 1 of the 2x2 set of graphs
ax3 = f2.add_subplot(222)  # adding graph 2 of the 2x2 set of graphs
ax4 = f2.add_subplot(223)  # adding graph 3 of the 2x2 set of graphs
ax5 = f2.add_subplot(224)  # adding graph 4 of the 2x2 set of graphs


plt.tick_params(labelcolor='none', top='off', bottom='off', left='off', right='off')
plt.grid(False)
plt.xlabel("LONGITUDE (E/W)")
ylbl = plt.ylabel("LATITUDE (N/S)")
ylbl.set_position([-.2, .5])
plt.title("GPS Coordinates")
plt.subplots_adjust(left=.125, right=0.9, top=.9, bottom=.1)


color_count = 0

ax.imshow(img, aspect= "auto", extent=[0, 10000, 0, 10000])
ax.autoscale(True)
ax.set_ylim([0,10000])
ax.set_xlim([0,10000])

#color_string = "black"

count = 0
count1 = 0

x = 0
y = 0
xtemp = 0     # old values of x, y, speed and altitude for the rover is stored here while the rover clears
ytemp = 0     # the plot and waits for new point. could do this for objective points too, probably should at some
altitude = 0  # point with an array for obj pts.
speed = 0
theta = 0      # instantaneous angle of movement on the rover (not inputted from GPS unit, rather we define it below)

objPtTempX= [0, 0, 0, 0, 0, 0]      # May use these arrays at some point to draw dotted lines from the rover
objPtTempY = [0, 0, 0, 0, 0, 0]     # location to each objective points (haven't done yet)

graph_page_clicked = False  # setting a global variable to keep track if the graphs page has been clicked,
                            # and once it is pressed the display functions start running to show the graphs.
                            # might end up changing the code format and deleting this variable.

def input_lat_and_long(lat, long):
    xList = []
    yList = []
    if lat == "":
        return
    if long == "":
        return
    lat = float(lat)
    long = float(long)

    xList.append(long)
    yList.append(lat)

    global color_count

    if (color_count == 0):
        color_string = "green"
    if (color_count == 1):
        color_string = "blue"
    if (color_count == 2):
        color_string = "red"
    if (color_count == 3):
        color_string = "purple"
    if (color_count == 4):
        color_string = "orange"
    if (color_count == 5):
        color_string = "#0AF524"
    if(color_count == 6):
        color_string = "#42DAED"
        color_count = 0

    objPtTempX.append(long)
    objPtTempY.append(lat)

    objpt = ax.scatter(xList, yList, color=color_string, s=50)
    objpt



def animate(self):  # function for animating the GPS coordinate movement
    global count

    pullData = open("C:/Users/Sean/PycharmProjects/tkinterFirst/URCROBOTICS/GPS/newGPSOutput1.dat", "r").read()
    dataList = pullData.split('\n')
    xList1 = []
    yList1 = []
    altList1 = []
    speedList1 = []


    global altitude
    global speed
    global theta  # Theta is angle of movement (compares old value to new value for x and y)
    global xtemp
    global ytemp
    global x
    global y
    for eachLine in dataList:
        if len(eachLine) > 31:
            x, y, z, z2 = eachLine.split('  ')  # these values are the most RECENT values from the GPS unit
            xList1.append(float(x))     # x is the LONGITUDE
            yList1.append(float(y))     # y is the LATITUDE
            altList1.append(float(z))   # z is the ALTITUDE
            speedList1.append(float(z2))    # z2 is the SPEED (knots, later converted to km/hr)

            x = float(x)
            y = float(y)

            xtemp = float(xtemp)
            ytemp = float(ytemp)

            if(y != ytemp and x!=xtemp):    # simple exception so that we don't get num/0 or 0/0
                if (is_number(x) and is_number(xtemp) and is_number(y) and is_number(ytemp)):
                    if ((x - xtemp) > 0 and (y - ytemp) > 0):
                        theta = math.atan(
                            (float(x) - float(xtemp)) / (float(y) - float(ytemp))) * 180/3.14159
                    if ((x - xtemp) > 0 and (y - ytemp) < 0):
                        theta = 180 - abs(math.atan((float(x) - float(xtemp)) / (float(y) - float(ytemp))) * 180/3.14159)
                    if ((x - xtemp) < 0 and (y - ytemp) > 0):
                        theta = math.atan(
                            (float(x) - float(xtemp)) / (float(y) - float(ytemp))) * (
                                                 180/3.14159)
                    if ((x - xtemp) < 0 and (y - ytemp) < 0):
                        theta = -180 + abs(math.atan(
                            (float(x) - float(xtemp)) / (float(y) - float(ytemp))) * (
                                                            180/3.14159))

            theta = round(theta, 2)
            xtemp = float(x)    # storing the old value of x for later comparison w/ new values of x
            ytemp = float(y)    # storing the old value of y for later comparison w/ new values of y
            altitude = float(z)     # storing altitude
            speed = float(z2) * 1.852   # z2 = speed which is multiplied by 1.852 to get in km/hr instead of knots
    paths = ax.scatter(xList1, yList1, s=100, color="white")  # plotting the longitude and latitude
    paths

    global color_count
    count =count + 1    # Need to routinely clear the plot so that the gps movement doesn't leave a "trail" of points
                        # so "count" is simply the counter for keeping track of how many times the point is plotted

    if (count > 20):    # clear the plot every 20 increments of "count", which increments every 0.1 sec (defined below)
        ax.clear()

        ax.imshow(img, aspect = "auto", extent=[0, 10000, 0, 10000])
        count = 0
        ax.set_ylim([0, 10000])
        ax.set_xlim([0, 10000])
        ax.scatter(xtemp,ytemp, color="white")

        if(is_number(obj_pt1.x_entry.get()) and is_number(obj_pt1.y_entry.get())):
            pt1Stats = objPtStats(pointStatsFrame, 1, float(obj_pt1.x_entry.get()), float(obj_pt1.y_entry.get()), xtemp,
                                  ytemp,
                                  0, 1, "#42DAED")
        if (is_number(obj_pt2.x_entry.get()) and is_number(obj_pt2.y_entry.get())):

            pt2Stats = objPtStats(pointStatsFrame, 2, float(obj_pt2.x_entry.get()), float(obj_pt2.y_entry.get()), xtemp,
                                  ytemp,
                                  1*4, 1, "blue")
        if (is_number(obj_pt3.x_entry.get()) and is_number(obj_pt3.y_entry.get())):

            pt3Stats = objPtStats(pointStatsFrame, 3, float(obj_pt3.x_entry.get()), float(obj_pt3.y_entry.get()), xtemp,
                                  ytemp,
                                  2 * 4, 1, "red")
        if (is_number(obj_pt4.x_entry.get()) and is_number(obj_pt4.y_entry.get())):

            pt4Stats = objPtStats(pointStatsFrame, 4, float(obj_pt4.x_entry.get()), float(obj_pt4.y_entry.get()), xtemp,
                                  ytemp,
                                  0 * 4, 2, "purple")
        if (is_number(obj_pt5.x_entry.get()) and is_number(obj_pt5.y_entry.get())):

            pt5Stats = objPtStats(pointStatsFrame, 5, float(obj_pt5.x_entry.get()), float(obj_pt5.y_entry.get()), xtemp,
                                  ytemp,
                                  1 * 4, 2, "orange")
        if (is_number(obj_pt6.x_entry.get()) and is_number(obj_pt6.y_entry.get())):

            pt6Stats = objPtStats(pointStatsFrame, 6, float(obj_pt6.x_entry.get()), float(obj_pt6.y_entry.get()), xtemp,
                                  ytemp,
                                  2 * 4, 2, "#0AF524")

        roverStats(roverStatsFrame, speed, altitude, theta, 2, 2)

        if(count1>0):
            if(is_number(set_dimensions.lowX.get()) and is_number(set_dimensions.highX.get()) and
                    is_number(set_dimensions.lowY.get()) and is_number(set_dimensions.highY.get())):
                apply_dims(set_dimensions.lowX.get(), set_dimensions.lowY.get(), set_dimensions.highX.get(), set_dimensions.highY.get())

    obj_pt1.button.invoke()
    color_count = 1
    obj_pt2.button.invoke()
    color_count = 2
    obj_pt3.button.invoke()
    color_count = 3
    obj_pt4.button.invoke()
    color_count = 4
    obj_pt5.button.invoke()
    color_count = 5
    obj_pt6.button.invoke()
    color_count = 6


def is_number(s):    #  imported a function to test if the values in the coordinate entries are actually numbers,
    try:             #  and not empty entries or entries with other symbols (e.g. $%@$^&\/!)
        float(s)     #  If an entry contains anything other than a number, the program ordinarily freezes so this
        return True  # is used to handle such cases
    except ValueError:
        pass

    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass
    return False


class obj_pt_set:   # make several instances of this class to add a objective pts (added to the frame 6 times for 6 pts)
    def __init__(self, theFrame, title, rowN, columnN, color_picked):

        self.textLabel = Label(theFrame, text=title, bg=color_picked, fg="white") # Label which obj pt we're talking abt
        self.x_entry = Entry(theFrame) # create an entry for the x-value of an objective point
        self.x_entry_title = Label(theFrame, text="x: ", bg=color_picked, fg="white") # title each entry
        self.x_entry.insert(0, "0") # set default x (longitude) value of 0 so that a null entry doesn't get graphed
                                    #  by default in the program

        self.y_entry = Entry(theFrame) # create an entry for the y-value of the objective point
        self.y_entry_title = Label(theFrame, text="y: ", bg=color_picked, fg="white") # Label the y-entry
        self.y_entry.insert(0, "0") # set default value of 0 for the y-value to avoid null entry trying to be graphed

        self.button = Button(theFrame, text="Submit", command=lambda: input_lat_and_long(self.y_entry.get(), self.x_entry.get()))
        # button to input x/y values (button isn't displayed but invoked during the program to display the obj. points)

        self.textLabel.grid(row=rowN, column=columnN, columnspan=2,  padx=5, pady=5)  # grid label w/ padding

        self.x_entry.grid(row=rowN+1, column=columnN+1)  # grid x-entry
        self.x_entry_title.grid(row=rowN + 1, column=columnN, padx=5, pady=5)  # grid the title

        self.y_entry.grid(row=rowN+2, column=columnN+1, padx=5, pady=5)  # grid the y-entry
        self.y_entry_title.grid(row=rowN + 2, column=columnN)  # grid the y-entry title


class objPtStats:  # this is the class for displaying an obj pt's stats, make one object for each point
    def __init__(self, frame, pointNum, xCoordPt, yCoordPt, xCoordRover, yCoordRover, rowN, columnN, color_picked):
        idealAngle = 0  # setting default "ideal angle" i.e. the angle you should move at to get to the point
                        # the quickest(see documentation for more details)
        eucDist = 0  # the euclidean distance (a.k.a. hypotenuse from longitude & latitude "sides" of triangle)
        xCoordPt = float(xCoordPt)  # x-coordinate of whichever objective point we're talking about
        yCoordPt = float(yCoordPt)  # y-coordinate of whichever objective point we're talking about
        xCoordRover = float(xCoordRover)  # x-coordinate of the rover at current point in time
        yCoordRover = float(yCoordRover)  # y-coordinate of the rover at current point in time
        if( (xCoordPt != xCoordRover) and (yCoordPt!= yCoordRover)): #  if the rover is not already at the point...
            if(is_number(xCoordPt) and is_number(xCoordRover) and (yCoordPt) and is_number(yCoordRover)):
            # if x-coordinate, y-coordinate of rover and objective point have values (i.e. are not null or whatever)

                # 4 different possibilities that determine how the angle is written for the 360 degree possibility
                # (see documentation for more details), and converts angle value from radians to degrees
                if((xCoordPt-xCoordRover) > 0 and (yCoordPt - yCoordRover) > 0):
                    idealAngle = math.atan((float(xCoordPt)-float(xCoordRover))/(float(yCoordPt)-float(yCoordRover))) * (180/3.14159)

                if((xCoordPt-xCoordRover) > 0 and (yCoordPt - yCoordRover) < 0):
                    idealAngle = 180 - abs(math.atan((float(xCoordPt)-float(xCoordRover))/(float(yCoordPt)-float(yCoordRover))) * (180/3.14159))

                if((xCoordPt-xCoordRover) < 0 and (yCoordPt - yCoordRover) > 0):
                    idealAngle = math.atan((float(xCoordPt)-float(xCoordRover))/(float(yCoordPt)-float(yCoordRover))) * (180/3.14159)

                if((xCoordPt-xCoordRover) < 0 and (yCoordPt - yCoordRover) < 0):
                    idealAngle = -180 + abs(math.atan((float(xCoordPt)-float(xCoordRover))/(float(yCoordPt)-float(yCoordRover))) * (180/3.14159))

                idealAngle = round(idealAngle, 2)  # round the ideal angle to two decimal places

                roverXKM = long_to_KM(xCoordRover) # call the long_to_KM() function (explained in func body) for rover
                roverYKM = lat_to_KM(yCoordRover)  # call the lat_to_KM() function (explained in func body) for rover

                # below we're calling the same function(s) for the objective point we're interested in
                ptXKM = long_to_KM(xCoordPt)
                ptYKM = lat_to_KM(yCoordPt)

                # calculating the euclidean distance from the hypotenuse in the difference in lat km and long km
                # between the rover and the objective point
                eucDist = math.sqrt((abs((ptXKM-roverXKM)*abs(ptXKM-roverXKM)) + abs((ptYKM-roverYKM)* abs(ptYKM-roverYKM))))
                eucDist = round(eucDist, 2)

        # creating a label for the objective point # --> "P", distance to rover from point -->"Dist (km) : ",
        # and the ideal angle --> "ideal ∠: "
        self.textLabel = Label(frame, text=("P" + str(pointNum)), bg = color_picked, fg="white")
        self.distToRover = Label(frame, text = ("Dist (km): " + str(eucDist)), bg = color_picked, fg="white")
        self.idealAngle = Label(frame, text = ("ideal ∠: " + str(idealAngle)), bg = color_picked, fg="white")

        # gridding the textLabel label, distToRover label, and idealAngle label with padding
        self.textLabel.grid(row=rowN, column=columnN, padx=20)
        self.distToRover.grid(row=rowN+1, column=columnN, padx=20)
        self.idealAngle.grid(row=rowN+2, column=columnN, padx=20, pady=5)

        # creating a separating line to separate the objective point stats, so that when we create multiple instances
        # of this class below, they'll be separated with a horizontal line
        tkinter.ttk.Separator(frame, orient=HORIZONTAL).grid(column=columnN, row=rowN+3, columnspan=2, sticky='ew')

class roverStats: # will put an instance of this class to display some important information about the rover status
    def __init__(self, frame, speed, altitude, theta, rowN, columnN):
        speed = round(speed, 2) # round the
        altitude  = round(altitude,2)

        self.speed_label = Label(frame, text=("speed (km/h): " + str(speed)), bg="grey")
        self.angle_label = Label(frame, text=("θ: " + str(theta)), bg="grey")
        self.altitude_label = Label(frame, text = "altitude: " + str(altitude), bg="grey")

        self.speed_label.grid(row=rowN, column=columnN)
        self.angle_label.grid(row=rowN+1, column=columnN)
        self.altitude_label.grid(row=rowN+2, column=columnN)


def lat_to_KM(a):  # converts the latitudinal units to kilometers (called by the ObjPtStats class)

    # variable "a" declared below is simply the latitude input, which will be converted to kilometers.
    # This conversion has no real meaning until we convert TWO points' latitude to kilometers
    # (i.e rover's latitude and an objective point's latitude), and take the difference
    # in kilometers to get one component of the Euclidean distance (other is longitude, for which there is a
    # similar function called long_to_KM() below)

    # below is the "algorithm" for breaking up the latitude information and convert its components to kilometers
    # The first two numbers in "a" are the input latitude DEGREES, and the remaining numbers are MINUTES (and decimals
    # of minutes). This is explained in more depth in GPS documentation (link below)
    # (https://docs.google.com/document/d/1MfLORGfgKrGZo_cjN4blhQnuVTPRbz1pzIjstgCNM3Y/edit)

    a = abs(a) * 10000
    aNum1 = a / 10000000
    aNum2 = (a % 10000000) / 1000000
    aNum3 = (a % 1000000) / 100000
    aNum4 = (a % 100000) / 10000
    aNum5 = (a % 10000) / 1000
    aNum6 = (a % 1000) / 100
    aNum7 = (a % 100) / 10
    aNum8 = a % 10
    aDeg = 10 * aNum1 + aNum2
    aMin = 10 * aNum3 + aNum4 + 0.1 * aNum5 + 0.01 * aNum6 + 0.001 * aNum7 + 0.0001 * aNum8
    aTotal = (111*aDeg + (111/60)*aMin)
    return aTotal


def long_to_KM(a):  # converts the longitudinal units to kilometers (called by the ObjPtStats class)
    a = abs(a) * 10000
    aNum1 = a / 10000000
    aNum2 = (a % 10000000) / 1000000
    aNum3 = (a % 1000000) / 100000
    aNum4 = (a % 100000) / 10000
    aNum5 = (a % 10000) / 1000
    aNum6 = (a % 1000) / 100
    aNum7 = (a % 100) / 10
    aNum8 = a % 10
    aDeg = 10 * aNum1 + aNum2
    aMin = 10 * aNum3 + aNum4 + 0.1 * aNum5 + 0.01 * aNum6 + 0.001 * aNum7 + 0.0001 * aNum8
    aTotal = (84*aDeg + (84/60)* aMin)
    return aTotal


class setDimensions:
    def __init__(self, frame, rowN, columnN):
        global count1
        self.lowX_label = Label(frame, text="min Long: ", bg = "grey")
        self.lowX = Entry(frame)

        self.lowY_label = Label(frame, text="min Lat: ", bg="grey")
        self.lowY = Entry(frame)


        self.highX_label = Label(frame, text="max Long: ", bg="grey")
        self.highX = Entry(frame)

        self.highY_label = Label(frame, text="max Lat: ", bg="grey")
        self.highY = Entry(frame)

        self.emergency_label = Label(frame, text = "EMERGENCY RESET:")
        self.emergency_button = Button(frame, text="CLICK ME!", command = lambda:set_bg_pringle())

        self.graph_page_label = Label(frame, text = "Real-time Graphs")
        self.graph_page_button = Button(frame, text = "open", command = lambda: open_scientific_graphs)

        if(count1==0):
            self.lowX.insert(0, "0")
            self.lowY.insert(0, "0")
            self.highX.insert(0, "0")
            self.highY.insert(0, "0")
            count1 = count1 +1

        self.lowX_label.grid(row=rowN, column = columnN)
        self.lowX.grid(row=rowN, column=columnN+1)

        self.highX_label.grid(row=rowN+1, column = columnN)
        self.highX.grid(row=rowN+1, column=columnN+1)

        self.lowY_label.grid(row=rowN+2, column=columnN)
        self.lowY.grid(row=rowN+2, column = columnN+1)

        self.highY_label.grid(row=rowN+3, column=columnN)
        self.highY.grid(row=rowN+3, column=columnN+1)

        self.emergency_label.grid(row=rowN+4, column=columnN)
        self.emergency_button.grid(row=rowN+5, column=columnN)

        self.graph_page_label.grid(row=rowN+4, column=columnN+1)
        self.graph_page_button.grid(row=rowN+5, column=columnN+1)


def set_bg_pringle():
    global img
    img = plt.imread("prangface.png")


def open_scientific_graphs(self):
    global graph_page_clicked
    global graphs_page
    global app
    #app.count_sec_UV = 0
    app.run_graphs()
    # ani2 = animation.FuncAnimation(f2, app.run_graphs, interval=200)
    #ax2.clear()

class graphPage:

    count_sec_UV = 0  # declaring static variable for UV sensor time (starts at time=0)
    def __init__(self, master, canvas):
        self.master = master
        self.canvas = canvas
        global f2
        myGraph2 = FigureCanvasTkAgg(f2, master=master)  # gets the graph to actually show on the canvas (else empty)
        myGraph2.get_tk_widget().grid(row=0, column=0)

    def run_graphs(self):
        '''ani_graph_1 = animation.FuncAnimation(f2, self.graph_animation_UV("C:/Users/Sean/Documents/URC_ROBOTICS/UV_sensor(SU-420)/ApogeeConnectLog.csv", self.ax2, self.master), interval=100) #  need to make this multi-threaded
        ani_graph_2 = animation.FuncAnimation(f2, self.graph_animation("C://Users//Sean//Documents//testOutput.txt", self.ax3), interval=100)
        ani_graph_3 = animation.FuncAnimation(f2, self.graph_animation("C://Users//Sean//Documents//testOutput.txt", self.ax4), interval=100)
        ani_graph_4 = animation.FuncAnimation(f2, self.graph_animation("C://Users//Sean//Documents//testOutput.txt", self.ax5), interval=100)'''

        self.graph_animation_UV()

    def graph_animation_co2(self, file_address, ax_num):
        pullData = open(file_address, "r").read()
        dataList = pullData.split('\n')
        xList = []
        yList = []
        start = time.time()
       #  ax4.clear()
        for eachLine in dataList:
            if len(eachLine) > 1 and len(eachLine) < 12:
                lolz, x = eachLine.split('  ')  # these values are the most RECENT values from the co2 sensor unit
                xList.append(float(x))     # x is the co2 ppm
                yList.append(self.count_sec_UV)

        ax4.scatter(yList, xList, color="white")  # plotting co2 ppm and time

        print ("my name is Ed")

    def graph_animation_UV(self):

        pullData = open("C:/Users/Sean/Documents/URC_ROBOTICS/UV_sensor(SU-420)/ApogeeConnectLog.csv", "r").read()
        dataList = pullData.split('\n')
        xList = []
        yList = []
        zList = []
        ax2.clear()
        for eachLine in dataList:
            if len(eachLine) >20:
                x, y, z = eachLine.split(',')  # these values are the most RECENT values from the GPS unit
                xList.append(x)  # date
                yList.append(self.count_sec_UV)  # time
                self.count_sec_UV = self.count_sec_UV + 1
                zList.append(float(z))  # value


        # self.ax2.clear()
        ax2.set_ylim([0, 0.7])

        ax2.plot(yList, zList, color="white")  # plotting the longitude and latitude
        self.graph_animation_co2("C:/Users/Sean/Documents/URC_ROBOTICS/GPS/CO2Output.txt", ax4)
        yList.clear()
        self.count_sec_UV = 0

        print("hello!")


def apply_dims(lowX, lowY, highX, highY):  # function to set the dimensions of the map (in lat/long)
    lowX = float(lowX)
    lowY = float(lowY)
    highX = float(highX)
    highY = float(highY)
    if (lowX > 0 and lowY > 0 and highX > 0 and highY > 0):
        ax.set_ylim([lowY, highY])
        ax.set_xlim([lowX, highX])


root = Tk()
graphs_page = tk.Toplevel()
root.geometry("1400x800")
root.configure(bg="#404040")

graph_page_clicked = True
canvas = Canvas(graphs_page, width=800, height=500)
app = graphPage(graphs_page, canvas)




myCanvas = Canvas(root, width=640, height=480, borderwidth=3, highlightbackground="white", highlightthickness=3)
myFrame = Frame(root, borderwidth=10, width=1000, height=240, bg="grey")
myFrame.config(highlightbackground="black", highlightthickness=5)
myCanvas2 = Canvas(root, width=360, height=240, bg="#508AAF")
myCanvas3 = Canvas(root, width=360, height=240, bg="#A18D5E")
pointStatsFrame = Canvas(root, width=240, height=240, bg="black")
roverStatsFrame = Canvas(root, width = 140, height = 240, borderwidth=4, bg = "grey")

myGraph = FigureCanvasTkAgg(f, master=myCanvas)
myGraph.get_tk_widget().pack(side="top", fill='both', expand=True)



obj_pt1 = obj_pt_set(myFrame, "Objective Point 1", 0, 0, "#42DAED")

obj_pt2 = obj_pt_set(myFrame, "Objective Point 2", 4, 0, "blue")
tkinter.ttk.Separator(myFrame, orient=VERTICAL).grid(column=2, row=0, rowspan=9, sticky='ns')

obj_pt3 = obj_pt_set(myFrame, "Objective Point 3", 0, 3, "red")
obj_pt4 = obj_pt_set(myFrame, "Objective Point 4", 4, 3, "purple")
tkinter.ttk.Separator(myFrame, orient=VERTICAL).grid(column=5, row=0, rowspan=9, sticky='ns')

obj_pt5 = obj_pt_set(myFrame, "Objective Point 5", 0, 6, "orange")
obj_pt6 = obj_pt_set(myFrame, "Objective Point 6", 4, 6, "#0AF524")
tkinter.ttk.Separator(myFrame, orient=HORIZONTAL).grid(column=0, row=3, columnspan=9, sticky='ew')

pt1Stats = objPtStats(pointStatsFrame, 1, float(obj_pt1.x_entry.get()), float(obj_pt1.y_entry.get()), xtemp,
                      ytemp,
                      0, 1, "#42DAED")
pt2Stats = objPtStats(pointStatsFrame, 2, float(obj_pt2.x_entry.get()), float(obj_pt2.y_entry.get()), xtemp,
                      ytemp,
                      1 * 4, 1, "blue")
pt3Stats = objPtStats(pointStatsFrame, 3, float(obj_pt3.x_entry.get()), float(obj_pt3.y_entry.get()), xtemp,
                      ytemp,
                      2 * 4, 1, "red")
pt4Stats = objPtStats(pointStatsFrame, 4, float(obj_pt4.x_entry.get()), float(obj_pt4.y_entry.get()), xtemp,
                      ytemp,
                      0 * 4, 2, "purple")
pt5Stats = objPtStats(pointStatsFrame, 5, float(obj_pt5.x_entry.get()), float(obj_pt5.y_entry.get()), xtemp,
                      ytemp,
                      1 * 4, 2, "orange")
pt6Stats = objPtStats(pointStatsFrame, 6, float(obj_pt6.x_entry.get()), float(obj_pt6.y_entry.get()), xtemp,
                      ytemp,
                      2 * 4, 2, "#0AF524")

add_rover_stats = roverStats(roverStatsFrame, speed, altitude, theta, 2, 2)
set_dimensions = setDimensions(roverStatsFrame, 6, 1)

myCanvas.grid(row=0, column=0, columnspan=3, rowspan=3)
pointStatsFrame.grid(row=0, column=3, columnspan=1, padx=4, pady=4, sticky="nsew")
myFrame.grid(row=3, column=0, columnspan=3)
roverStatsFrame.grid(row=2, column=3, columnspan=1, padx=4, pady=4, sticky="nsew")
myCanvas2.grid(row=0, column=4)
myCanvas3.grid(row=2, column=4)

if(graph_page_clicked):
    ani2 = animation.FuncAnimation(f2, open_scientific_graphs, interval=1000)
ani = animation.FuncAnimation(f, animate, interval=200)
root.mainloop()
