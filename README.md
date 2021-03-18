# pae_sim

https://github.com/d-roma/pae_sim/

## Requisits del python
- Python3
- pyserial

##Estructura del projecte

1. AX.py: classes per l'emulació de les memories dels motors AX12 i del sensor AXS1. 
2. com.py: classe per gestionar les communicacions amb el robot, emulant el protocol i accedint a les memories AX
3. emulador_robot_PAE.py: Main a executar. Detallat després.
4. global_config.py: Fitxer amb parametres compartits entre els fitxers del emulador.
5. gui.py: Aplicació grafica. S'encarrega també d'obrir la comunicació amb la UART i d'anar executant periodicament l'actualització del emulador
6. plot_movement.py: Ploteja l'habitació i la posició del robot
7. sim.py: El simulador propiament del moviment del robot
8. world.py: Carrega el fitxer de l'habitació

L'emulador_robot_PAE primer de tot genera un segon proces per dibuixar l'habitació i la posició del robot. Després, crea les memories, crida a World per llegir l'habitació i inicialitza la simulació amb l'habitació. Finalment, crida l'aplicació gràfica, que servirà per configurar diferents nivells de debug. A més te dos altres funcions importants:
- Al seleccionar un port (o si existeix el port per defecte) crea un nou fil per la comunicació
- S'anirà executant el mainloop, que tractarà els events de l'aplicació gràfica (actualització de valors i lectura dels canvis) i també anirà cridant el simulador per actualitzar l'estat. 

En cas que sorti d'aquest bucle infinit, es tanca també el procés secundari encarregat de fer el plot del moviment. 


