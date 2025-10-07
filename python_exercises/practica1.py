# %% [markdown]
# # Practical Session 2: Introduction to Python
# 
# ## Laboratorio de Robótica 
# ### Grado en Ingeniería Electrónica, Mecatrónica y Robótica
# ### Universidad de Sevilla

# %% [markdown]
# ## Objectives
# 
# The objective of this notebook is to propose some exercises to practise with Python and get familiar with data types, loops, functions, classes, and so on.
# 
# After completing the missing code in this notebook, export it as a Python script. You can do that both using Jupyther Notebook or through the Jupyter extension in VS Code. Then name your file `exercises_sol.py` and commit it to your course Github repository, into a folder named `python_exercises`.

# %% [markdown]
# ### Exercise 1
# 
# Define a function called `squares` that, given a list `sec` of numbers, returns a list with the squares of those numbers, in the same order.

# %%
# Examples:

# squares([2, -1.2, 3e2, 1j]) should return [4, 1.44, 90000.0, (-1+0j)]
# squares([i for i in range(10)]) should return [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]

# %%
def squares(l):
    sqr=[]
    for i in l:
        sqr.append(i**2)
    return sqr

squares([2,3,4,(2+3j)])


# %% [markdown]
# ### Exercise 2
# 
# A positive integer is said to be perfect if it coincides with the sum of all its proper divisors (that is, other than itself). Define a function called `write_perfect` that, given two positive integers `m` $\leq$ `n`, returns a list with all the perfect numbers within the interval `[m, n]`. The function should also print on the screen each perfect number and its divisors. 

# %%
# Examples:

# write_perfect(1, 1000) should write on screen:
# Number 6 is perfect and its divisors are [1, 2, 3]
# Number 28 is perfect and its divisors are [1, 2, 4, 7, 14]
# Number 496 is perfect and its divisors are [1, 2, 4, 8, 16, 31, 62, 124, 248]

# %%
def write_perfect(m,n):
    for i in range(m,n):
        lista=[]
        suma=0
        perfecto=False
        for x in range(1,i):
            if i % x == 0:
                lista.append(x)
                suma +=x

        if suma==i:
            print("Number " + str(i) + " is perfect and its divisor are " + str(lista))

write_perfect(0, 10000)

# %% [markdown]
# ### Exercise 3
# 
# Consider a dictionary whose keys are character strings of length one and the associated values ​​are non-negative integers, such as the following dictionary `d`:

# %%
d = {'a': 5, 'b': 10, 'c': 12, 'd': 11, 'e': 15, 'f': 20, 'g': 15, 'h': 9, 'i': 7, 'j': 2}

# %% [markdown]
# Define a function called `horizontal_histogram` that, given a dictionary of the previous type, writes the associated histogram of horizontal bars on the screen, printing the bars from top to bottom in the order determined by the `sorted` function on the keys, as illustrated in the following example:

# %%
# horizontal_histogram(d)
# a: *****
# b: **********
# c: ************
# d: ***********
# e: ***************
# f: ********************
# g: ***************
# h: *********
# i: *******
# j: **

# %%


dict_tel = {'a': 5, 'b': 10, 'c': 12, 'd': 11, 'e': 15, 'f': 20, 'g': 15, 'h': 9, 'i': 7, 'j': 2}


def horizontal_histogram(hist):

    llaves= list(dict_tel.keys())

    ast = "*"
    sentence= " "

    for i,k in hist.items():
    
        for l in range(k):
            sentence += ast
            
        
        letra=llaves[llaves.index(i)]

        print( letra + ":" + sentence )

        sentence= " "    
    return


horizontal_histogram(dict_tel)



# %% [markdown]
# ### Exercise 4
# 
# Suppose we want to simulate the trajectory of a drone that is launched at a given point with a certain initial height. The drone is launched forward with an initial speed and at a certain angle without propulsion motors on. Initially it will advance upwards, but due to the force of gravity, at a given moment it will begin to descend until it lands. For simplicity, we will assume that there is no friction or wind resistance.
# 
# Define a class `Drone` that represents the state of the drone at a given instant of time. At least, the class should include attributes to store the following data:
# + Traveled distance traveled (horizontally)
# + Height
# + Horizontal speed
# + Vertical speed
# 
# In addition, apart from its constructor, the class should have the following three methods:
# + `get_pos_x`: it returns the horizontal traveled distance 
# + `get_pos_y`: it returns the vertical traveled distance 
# + `update_position`: given a number `t` of seconds, it updates the position and velocity of the projectile after that time has elapsed
# 
# Once the `Drone` class is defined, define an external function called `land` that, given the `height` (meters), `velocity` (meters per second), `angle` (degrees) and time `step` (seconds), prints on the screen the different positions of a drone launched with that initial `height`, `velocity` and `angle`. The position of the drone should be displayed at each `step` of time, until it lands. The function should also print the maximum height reached by the drone, the total distance traveled horizontally and the time and number of steps that it took it to land.
# 
# Indications:
# 1. If the drone has an initial velocity $v$ and is launched at an angle $\theta$, the horizontal and vertical components of the initial velocity are $v \times \cos(\theta)$ and $v \times \ sin(\theta)$, respectively.
# 2. The horizontal component of velocity, in the absence of friction and wind, will remain constant.
# 3. The vertical component of the velocity evolves throughout time: if $vy_0$ is the initial vertical velocity, after a time step $t$, the velocity will be $vy_1 = vy_0 - 9.8 \times t$, due to the Earth's gravity.
# 4. Also, if $h_0$ is the initial drone height, after a time step $t$, the height will be $h_1 = h_0 + vm \times t$, where $vm$ is the average between the previous $vy_0$ and $vy_1$.

# %%
# Example:

# land(30, 1, 20, 0.1)
# Drone at position(0.0, 30.0)
# Drone at position(0.1, 30.0)
# Drone at position(0.2, 29.9)
# Drone at position(0.3, 29.7)
# Drone at position(0.4, 29.4)
# Drone at position(0.5, 28.9)
# Drone at position(0.6, 28.4)
# Drone at position(0.7, 27.8)
# Drone at position(0.8, 27.1)
# Drone at position(0.8, 26.3)
# Drone at position(0.9, 25.4)
# Drone at position(1.0, 24.4)
# Drone at position(1.1, 23.4)
# Drone at position(1.2, 22.2)
# Drone at position(1.3, 20.9)
# Drone at position(1.4, 19.5)
# Drone at position(1.5, 18.0)
# Drone at position(1.6, 16.4)
# Drone at position(1.7, 14.7)
# Drone at position(1.8, 13.0)
# Drone at position(1.9, 11.1)
# Drone at position(2.0, 9.1)
# Drone at position(2.1, 7.0)
# Drone at position(2.2, 4.9)
# Drone at position(2.3, 2.6)
# Drone at position(2.3, 0.2)

# After 26 steps of 0.1 seconds (2.6 seconds), the drone has landed.
# It has traveled a distance of 2.4 meters.
# It has reached a maximum height of 30.0 meters.

# %%
import math

class Drone():
    def __init__(self, x=0, y=0, vx=0, vy=0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        
    def get_pos_x(self,x):
        self.x = x
        return
    
    def get_pos_y(self, y):
        self.y = y
        return
    
    def update_position(self,t):

        self.x += self.vx * t

        vy_1 = self.vy - 9.8 * t

        vm = (self.vy + vy_1)/2

        y_1 = vm * t

        self.y += y_1

        self.vy = vy_1

        return [self.x, self.y, self.vx, self.vy]
    

def land(heigh,velocity,angle,step):

    t = 0.0

    x_new=0

    angulo = math.radians(angle)

    vx = velocity * math.cos(angulo)

    vy = velocity * math.sin(angulo)

    dron_1 = Drone(0,heigh,vx,vy)

    y = float(heigh)

    y_new = heigh

    y_max = y

    print( "Drone at position (" + str(t) + ", " + str(y) + ")" )

    while y_new>0:
        pos_vel = dron_1.update_position(step)

        x_new = pos_vel[0]
        y_new = pos_vel[1]

        if y_new<0:
            break

        dron_1.get_pos_x(x_new)
        dron_1.get_pos_y(y_new)

        t += step

        if y_new > y_max:
            y_max = y_new

        print(f"Drone at position ({x_new:.2f}, {y_new:.2f})" )


    print(f"\nAfter {t/step:.0f} steps of {step} seconds ({t:.2f} seconds), the drone has landed" )
    print(f"It has traveled a distance of {x_new:.3f} metres" )
    print(f"It has reached a maximum heigh of {y_max:.2f} metres" )


    return

land(30, 1, 80, 0.1)


# %% [markdown]
# ### Exercise 5
# 
# Define a function called `matrix_operation` that receives an integer `n` as argument. The function should create a NumPy array (vector) with the integers within the interval $[n,n+25)$ and perform the following operations:
# 
# + Calculate the mean and standard deviation of the array and print it on the screen.
# + Reshape the array into a 5x5 matrix, calculate the determinant of the matrix and print the result on the screen.
# + Return a tuple with the three computed values `(mean, std, determinant)`.

# %%
# Example

# matrix_operation(1)
# The mean and standard deviation of the vector is 13.0 +/- 7.211102550927978.
# The determinant of the matrix is 0.0.

# %%
import math
import numpy as np

def matrix_operation(n):
    vector=[]
    suma=0
    total=0
    for i in range(n,n+25):
        vector.append(i)
        suma+=i
        total+=1

    #calculamos la media
    media=suma/total

    media=np.mean(vector)

    #calculamos la desviación típica
    desviacion = np.std(vector)
    print("The mean and the standart desviation of the vector is " + str(media) + " +/- " + str(desviacion))

    matriz=np.matrix([[n,n+1,n+2,n+3,n+4],[n+5,n+6,n+7,n+8,n+9],[n+10,n+11,n+12,n+13,n+14],[n+15,n+16,n+17,n+17,n+19], [n+20,n+21,n+22,n+23,n+24]])
        
    determinante=np.round(np.linalg.det(matriz),2)
    print(f"The determinant of the matrix is {determinante:.1f}")

    tuple=(float(media),float(desviacion), float(determinante)) 
    return tuple



matrix_operation(1)

# %% [markdown]
# ### Exercise 6
# 
# Create a function called `plot_functions` that makes a figure and plot the following functions over the range $[0, 10]$:
# 
# + $y_1 = sin(x)$
# + $y_2 = cos(x)$
# 
# The figure should also include appropriate labels, title, and a legend.

# %%
import math
import numpy as np
import matplotlib.pyplot as plt

def plot_functions():
    x=np.linspace(0,10,1000)
    y1=np.sin(x)
    y2=np.cos(x)
    plt.plot(x, y1,x,y2)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Gráfica seno y coseno')
    plt.show()


plot_functions()


