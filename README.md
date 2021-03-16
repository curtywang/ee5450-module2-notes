# ee5450-module2-hw0
Homework on resource control and interrupt handling with ThreadX

In this homework, you will practice using the atomic data structures (or synchronization primitives)
to protect the GPIO pins, along with basic ThreadX application setup.

## ThreadX Compilation Setup
Since we are using CMake, the updated CMakeLists_template.txt should be able to add threadx as a subdirectory.
However, you will need to go into your CLion settings and change the CMake settings under "Build, Execution, Deployment".
In "CMake options:", add `-GNinja`, which will tell CMake to use the Ninja tool that ThreadX requires.

## Thread 0: LED1 Blinker
This thread is simple.  Simply blink LD1 (PA_5) at a frequency of 1 second with a duty cycle of 50%.

## Thread 1: LED2 Blinker
For this thread, you will implement a double-flash on LD2 (PB_14).  This double-flash
will repeat at a 1 second interval.  Each flash will be 100 ms long.  Basically,
this thread should just be composed of an infinite loop of the following:
1. Turn LED on
2. Wait 100 ms
3. Turn LED off
4. Wait 100 ms
5. Turn LED on
6. Wait 100 ms
7. Turn LED off
8. Wait 700 ms

## Thread 2: Button Handler
This thread will turn on the LED when BUTTON1 (PC_13) is pressed.  Note that
this button uses negative logic (it is LOW when pressed).
To implement this thread, use a mutex to guard access to LD2 (PB_14). 
Make sure to get and set the mutex in both Thread 1 and Thread 2, and make sure 
to create the mutex in `tx_application_define()`.  This thread will turn LED2
on if the button is pressed, and keep LED2 on for as long as the button is 
held down.

To implement this button handling, take advantage of the event flag structure.
Make sure to debounce (simply just sleep 20 milliseconds after the event flag lets you through).
Wait until an event happens to decide whether to:
1. Turn on LED2 if BUTTON1 is pressed: get access to LED2 using the mutex and turn LED2 on.
2. Turn off LED2 if BUTTON1 is released: turn off LED2 and release access to LED2.
