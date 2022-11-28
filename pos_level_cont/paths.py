# function for definition of the paths
import numpy as np

def path_pars(t,t_end,c,tilt,rd_init, shape):
    ## Inputs
    # t - current time, 
    # t_end - time taken to complete the path,
    # c - size of the shape, 
    # tilt - angle made by path with xy plane
    # rd_int = starting position for path 
    # shape - shape of path

    ## Outputs
    # rd -  desired position for the path ,
    # rd_dot - desired velocity for the path, 
    # rd_ddot - desired acceleration for path
    
    if shape == 'cardioid':
    #cardioid path
        n = t_end/10
        t = t/n
        rd = np.array([2*c*np.sin(0.2*np.pi*t)*np.cos(tilt)-c*np.sin(0.4*np.pi*t)*np.cos(tilt)+rd_init[0],
                    2*c*np.cos(0.2*np.pi*t)-c*np.cos(0.4*np.pi*t)+rd_init[1]-c,
                    2*c*np.sin(0.2*np.pi*t)*np.sin(tilt)-c*np.sin(0.4*np.pi*t)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([2*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.cos(tilt)-c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.cos(tilt),
                        -2*c*0.2*np.pi*np.sin(0.2*np.pi*t)+c*0.4*np.pi*np.sin(0.4*np.pi*t),
                        2*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.sin(tilt)-c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.sin(tilt)])
        
        rd_ddot =np.array([-2*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.cos(tilt)+c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.cos(tilt),
                        -2*c*((0.2*np.pi)**2)*np.cos(0.2*np.pi*t)+c*((0.4*np.pi)**2)*np.cos(0.4*np.pi*t),
                        -2*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.sin(tilt)+c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt)])
    
    elif shape == 'adobe':
        # epicycloid path
        n = t_end/10
        t = t/n
        rd = np.array([-3*c*np.sin(0.2*np.pi*t)*np.cos(tilt)+2*c*np.sin(0.4*np.pi*t)*np.cos(tilt)+rd_init[0],
                   3*c*np.cos(0.2*np.pi*t)+2*c*np.cos(0.4*np.pi*t)+rd_init[1]-5*c,
                   -3*c*np.sin(0.2*np.pi*t)*np.sin(tilt)+2*c*np.sin(0.4*np.pi*t)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([-3*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.cos(tilt)+2*c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.cos(tilt),
                    -3*c*0.2*np.pi*np.sin(0.2*np.pi*t)-2*c*0.4*np.pi*np.sin(0.4*np.pi*t),
                    -3*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.sin(tilt)+2*c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.sin(tilt)])
    
        rd_ddot =np.array([-3*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.cos(tilt)-2*c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.cos(tilt),
                    +3*c*((0.2*np.pi)**2)*np.cos(0.2*np.pi*t)-2*c*((0.4*np.pi)**2)*np.cos(0.4*np.pi*t),
                    -3*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.sin(tilt)-2*c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt)])
    

   

    elif shape == 'hyp':
        # hypocycliod path
        n = t_end/10
        t = t/n
        rd = np.array([2*c*np.sin(0.2*np.pi*t)*np.cos(tilt) - c*np.sin(0.8*np.pi*t)*np.cos(tilt)+rd_init[0],
                    2*c*np.cos(0.2*np.pi*t) - c*np.cos(0.8*np.pi*t)+rd_init[1]-c,
                    2*c*np.sin(0.2*np.pi*t)*np.sin(tilt)- c*np.sin(0.8*np.pi*t)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([2*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.cos(tilt) - c*0.8*np.pi*np.cos(0.8*np.pi*t)*np.cos(tilt),
                        -2*c*0.2*np.pi*np.sin(0.2*np.pi*t) + c*0.4*np.pi*np.sin(0.8*np.pi*t),
                        2*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.sin(tilt) - c*0.4*np.pi*np.cos(0.8*np.pi*t)*np.sin(tilt)])
        
        rd_ddot =np.array([-2*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.cos(tilt) + c*((0.8*np.pi)**2)*np.sin(0.8*np.pi*t)*np.cos(tilt),
                        -2*c*((0.2*np.pi)**2)*np.cos(0.2*np.pi*t) + c*((0.8*np.pi)**2)*np.cos(0.8*np.pi*t),
                        -2*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.sin(tilt) + c*((0.8*np.pi)**2)*np.sin(0.8*np.pi*t)*np.sin(tilt)])
        


    elif shape == 'petal':
        n = t_end/5
        t = t/n
        rd = np.array([c*np.sin(0.4*np.pi*t)*np.cos(tilt)-c*np.sin(1.2*np.pi*t)*np.cos(tilt)+rd_init[0],
                   c*np.cos(0.4*np.pi*t)+ c*np.cos(1.2*np.pi*t)+rd_init[1]-2*c,
                   c*np.sin(0.4*np.pi*t)*np.sin(tilt) - c*np.sin(1.2*np.pi*t)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.cos(tilt) - c*1.2*np.pi*np.cos(1.2*np.pi*t)*np.cos(tilt),
                       -c*0.4*np.pi*np.sin(0.4*np.pi*t)- c*1.2*np.pi*np.sin(1.2*np.pi*t),
                       c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.sin(tilt) - c*1.2*np.pi*np.cos(1.2*np.pi*t)*np.sin(tilt)])
    
        rd_ddot =np.array([-c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.cos(tilt)+ c*((1.2*np.pi)**2)*np.sin(1.2*np.pi*t)*np.cos(tilt),
                    -c*((0.4*np.pi)**2)*np.cos(0.4*np.pi*t) - c*((1.2*np.pi)**2)*np.cos(1.2*np.pi*t),
                    -c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt) + c*((1.2*np.pi)**2)*np.sin(1.2*np.pi*t)*np.sin(tilt)])

    elif shape == 'tricuspid':

        n = t_end/10
        t = t/n
        rd = np.array([-2*c*np.sin(0.2*np.pi*t)*np.cos(tilt)+c*np.sin(0.4*np.pi*t)*np.cos(tilt)+rd_init[0],
                    2*c*np.cos(0.2*np.pi*t)+c*np.cos(0.4*np.pi*t)+rd_init[1]-3*c,
                    -2*c*np.sin(0.2*np.pi*t)*np.sin(tilt)+c*np.sin(0.4*np.pi*t)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([-2*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.cos(tilt)+c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.cos(tilt),
                        -2*c*0.2*np.pi*np.sin(0.2*np.pi*t)-c*0.4*np.pi*np.sin(0.4*np.pi*t),
                        -2*c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.sin(tilt)+c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.sin(tilt)])
        
        rd_ddot =np.array([-2*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.cos(tilt)-c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.cos(tilt),
                        +2*c*((0.2*np.pi)**2)*np.cos(0.2*np.pi*t)-c*((0.4*np.pi)**2)*np.cos(0.4*np.pi*t),
                        -2*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.sin(tilt)-c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt)])

    elif shape == 'circle':
        # circle
        n = t_end/10
        t = t/n
        rd = np.array([c*np.sin(0.2*np.pi*t)*np.cos(tilt)+rd_init[0],
                   c*np.cos(0.2*np.pi*t)+rd_init[1]-c,
                   c*np.sin(0.2*np.pi*t)*np.sin(tilt)+rd_init[2]])

 
        rd_dot = np.array([c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.cos(tilt),
                     -c*0.2*np.pi*np.sin(0.2*np.pi*t),
                    c*0.2*np.pi*np.cos(0.2*np.pi*t)*np.sin(tilt)])
        rd_ddot =np.array([-c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.cos(tilt),
                    -c*((0.2*np.pi)**2)*np.cos(0.2*np.pi*t),
                    -c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.sin(tilt)])

    
    

    #star shape
    elif shape == 'star':
        n = t_end/10
        t = t/n

        rd = np.array([c*np.sin(0.6*np.pi*t)*np.cos(tilt)-2*c*np.sin(0.4*np.pi*t)*np.cos(tilt)+rd_init[0],
                    c*np.cos(0.6*np.pi*t)+2*c*np.cos(0.4*np.pi*t)+rd_init[1]-3*c,
                    c*np.sin(0.6*np.pi*t)*np.sin(tilt)-2*c*np.sin(0.4*np.pi*t)*np.sin(tilt)+rd_init[2]])
        rd_dot = np.array([c*0.6*np.pi*np.cos(0.6*np.pi*t)*np.cos(tilt)-2*c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.cos(tilt),
                        -c*0.6*np.pi*np.sin(0.6*np.pi*t)-2*c*0.4*np.pi*np.sin(0.4*np.pi*t),
                        c*0.6*np.pi*np.cos(0.6*np.pi*t)*np.sin(tilt)-2*c*0.4*np.pi*np.cos(0.4*np.pi*t)*np.sin(tilt)])
    
        rd_ddot =np.array([-c*((0.6*np.pi)**2)*np.sin(0.6*np.pi*t)*np.cos(tilt)+2*c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.cos(tilt),
                        -c*((0.6*np.pi)**2)*np.cos(0.6*np.pi*t)-2*c*((0.4*np.pi)**2)*np.cos(0.4*np.pi*t),
                        -c*((0.6*np.pi)**2)*np.sin(0.6*np.pi*t)*np.sin(tilt)+2*c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt)])
        

    elif shape == 'lissajous':
        rd = np.array([4*c*np.sin(0.5*np.pi*t)*np.cos(tilt) + rd_init[0],
                       3*c*np.sin(np.pi*t)+rd_init[1],
                       4*c*np.sin(0.5*np.pi*t)*np.sin(tilt) + rd_init[2]])

        rd_dot = np.array([4*c*(0.5*np.pi)*np.cos(0.5*np.pi*t)*np.cos(tilt),
                           3*c*(np.pi)*np.cos(np.pi*t),
                           4*c*(0.5*np.pi)*np.cos(0.5*np.pi*t)*np.sin(tilt)])
    
        rd_ddot = np.array([-4*c*((0.5*np.pi)**2)*np.sin(0.5*np.pi*t)*np.cos(tilt),
                           -3*c*((np.pi)**2)*np.sin(np.pi*t),
                           -4*c*((0.5*np.pi)**2)*np.sin(0.5*np.pi*t)*np.sin(tilt)])


    else:
        print('shape name not recognized')

    return rd, rd_dot, rd_ddot