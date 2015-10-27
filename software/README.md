# STM32L476-radar software

Eclipse project with all the sources for the radar software used in the demo.

You'll need to be using the Eclipse IDE for C/C++ with the [AC6 System Workbench](http://www.openstm32.org/System+Workbench+for+STM32) Eclipse plugin installed.

From Eclipse, right click in the project pane and select Import->General->Existing Project and choose the STM32-radar folder.

It'll build two artefacts: `STM32-radar.elf` and `STM32-radar.bin`, and then try and download the binary file to the device using the `st-flash`.


