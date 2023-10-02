#Program for optimizing (minimizing) using our algorithm of "random directions".
#usage: coeffmin=optimrandomdir(objfunction, coeffs, args=(X,))
#arguments: objfunction: name of objective function to minimize
#coeffs: Starting vector for the optimization, also determines the dimensionality of the input for objfunction
#Can be a tensor of any dimension!
#X: further arguments for objfunction
#With line search for successful directions
#For time varying objective functions, hence x0 is computed every time in the beginning of the optimizazion loop.
#returns: coeffmin: coeff vector which minimizes objfunction
#Gerald Schuller, August 2021

import numpy as np

def objfunccomp(objfunction, coeffs, args, bounds):
   #compact objective function for optimization
   if bounds !=():
         for n in range(len(bounds)):
            coeffs[n]=np.clip(coeffs[n],bounds[n][0], bounds[n][1])
   if len(args)==0:
      X0=objfunction(coeffs)
   elif len(args)==1:
      X0=objfunction(coeffs, args[0])
   else:
      X0=objfunction(coeffs, args)
   return X0
   

def optimrandomdir(objfunction, coeffs, args=(), bounds=(), coeffdeviation=1.0, iterations=1000, startingscale=2, endscale=0.0):
   #Reads in a block of a stereo signal, improves the unmixing coefficients,
   #applies them and returns the unmixed block
   #Arguments: objfunction: (char string) name of objective function to minimize
   #coeffs: array of the coefficients for which to optimize
   #args: Additional arguments for objfunction, must be a tuple! (in round parenthesis)
   #bounds: bounds for variables or coefficients, sequence (Tuple or list) of pairs of minima and maxima for each coefficient (optional)
   #This sequence can be shorter than the coefficent array, then it is only applied to the first coefficients that the bounds cover.
   #This can be used to include side conditions. Example: bounds=((0,1.5),(-1.57,1.57))
   #coeffdeviation: array of the expected standard deviation of the coefficients
   #iterations: number of iterations for optimization
   #startingscale: scale for the standard deviation for the random numbers at the beginning of the iterations 
   #endscale: scale of the random numbers et the end of the iterations.
   #returns: 
   #Xunm: the unmixed stereo block resulting from the updated coefficients 
   #coeffsmin: The coefficients which minimize objfunction
   
   
   #print("args=", args, "args[0]=", args[0])
   coeffs=np.array(coeffs) #turn into numpy array (if not already)
   sh=coeffs.shape #shape of objfunction input
   #Simple online optimization, using random directions optimization:
   #Initialization of the deviation vector:
   #print("coeffdeviation=", coeffdeviation)

   try:
      if coeffdeviation==1.0:
         coeffdeviation=np.ones(sh)*1.0
   except:
      print("use specified coefficient std. deviations")


   """
   if len(args)==0:
      X0=objfunction(coeffs)
   elif len(args)==1:
      X0=objfunction(coeffs, args[0])
   else:
      X0=objfunction(coeffs, args)
   """
   """
   argstr=''
   for i in range(len(args)):
      argstr+=',args['+str(i)+']'
   argstr=argstr[1:]
   print('argstr=', argstr)
   print("eval(argstr)=", eval(argstr))
   #print("Call=", objfunction+'(coeffs'+argstr+')')
   #X0=eval(objfunction+'(coeffs'+argstr+')')
   #X0=objfunction(coeffs, eval(argstr))
   X0=objfunction(coeffs, args)
   """
   
   #Small random variation of coefficients, -0.05..0.05 for attenuations, -0.5..0.5 for delays:
   #coeffdeviation=np.array([0.1,0.1,1.0,1.0])*0.8
   #setfrac=8/max(sh) #0.0 <= setfrac <=1.0, 
               #probability of a coefficent to be updated, for reducing dimensionality. 
               #1.0: all coeffs are updated
               #A simple way to create subspaces of lower dimensionality
               #if setfrac is too small it easily gets stuck in local minima.
   #print("sh*2=", sh*2)
   #subspacematrix=np.random.normal(loc=0.0, scale=1.0, size=sh*2) #random matrix of size sh x sh, subspace approach
   m=0; mlastupdate=0
   #iterations=300  #for the optimization, 100000 for LDFB, set below as global variable
   #scale=1.0  #standard deviation for the gaussian random number generator
   scalehistsize=10
   scalehistory=np.zeros((scalehistsize,2))
   alpha=1/(2*len(coeffs)) #update factor for scale
   #alpha=1/(2*8)
   succ=False
   mean=np.zeros(sh)
   for m in range(iterations):
   #while (np.linalg.norm(coeffdeviation)>1e-6):
      #Old values 0, starting point:
      X0=objfunccomp(objfunction, coeffs, args, bounds)
      setfrac=4/max(sh) #take subspace of about 8 or 4 coefficients
      #setfrac=max(8.0/max(sh),8.0/iterations) #take larger subspaces such that larger dimensions can be fully covered more easily
      #setfrac=np.random.rand(1) #random fraction of coefficients
      #scale=np.random.rand(1); #random scale (std. deviation for gauss distribution)
      #setfrac=np.clip(1.0-m/iterations,1/sh[0],1) #begin with global optimization, later fine tune coeff in lower dimensions
      #setfrac=np.clip(1.0-m/iterations,np.sqrt(8.0/max(sh)),1)**2 #begin with global optimization, later fine tune coeff in lower dimensions
      #setfrac=np.clip(1.0-m/iterations,8.0/max(sh),1) #begin with global optimization, later fine tune coeff in lower dimensions
      #setfrac=np.clip(m/iterations,1/sh[0],1) #begin with small dimension, later fine tune coeff in higher dimensions
      #setfrac=setfrac **2 #become smaller faster
      #setfrac=setfrac ** 0.5 #become faster slower
      #scale=abs(np.random.normal(loc=0.0, scale=1.0, size=1)[0])
      #scale=np.random.rand(1)[0]
      #scale=((1.0-m/iterations))
      #scale=(1.4*(1.0-m/iterations))**2
      scale=np.clip((startingscale-endscale)*((1.0-1.0*m/iterations)**2)+endscale,1e-4,None)  #scale becomes smaller when iterations icrease (so far best)
      #scale=np.clip((startingscale-endscale)*(np.sin(6.28/iterations*10*m)**2)+endscale,1e-4,None)  #scale oscillates with iterations 
      #scale=4.0*np.exp(-8*m/iterations)
      #scale*=0.9999  #exponential decay of scale of random numbers
      if m%1000==0:
         print("m=", m, "setfrac=", setfrac, "scale=", scale ); m+=1
      #print("sh*2=", sh*2)
      #coeffvariation=(np.random.rand(4)-0.5)*coeffdeviation
      #coeffvariation=4*(np.random.random(sh)-0.5)*coeffdeviation  #small variation, uniform distribution
      #coeffset=np.ones(coeffs.shape)
      coeffset=np.random.random(coeffs.shape)<=setfrac #Set of coefficients to be updated
      #print("coeffdeviation=", coeffdeviation)
      #if succ==False:
      coeffvariation=np.random.normal(loc=mean, scale=scale*coeffdeviation) #Gaussian distribution
      coeffvariation *= coeffset #keep only the variation in the subset, for sparse coefficients
      #coeffvariation=np.dot(subspacematrix,coeffvariation)  #subspace approach
      #print("coeffvariation=", coeffvariation)
      #new values 1:
      """
      if args==():
         X1=eval(objfunction+'(coeffs+coeffvariation)')
      else:
         X1=eval(objfunction+'(coeffs+coeffvariation, args)')
      """
      #print('argstr=', argstr)
      #X1=eval(objfunction+'(coeffs+coeffvariation'+argstr+')')
      #X1=objfunction(coeffs+coeffvariation, args)
      c1=coeffs+coeffvariation
      
      #Here possibly loop over a set for parallel processing,
      #and X1 as the lowest over the set.
      X1=objfunccomp(objfunction, c1 , args, bounds)
      
      if X1<X0:  #New is better
         #print("last was succ:", succ)
         succ=True
         X0=X1
         #line search:
         #first towards larger:
         for lineexp in range(1,9): #-4,6
            X0=objfunccomp(objfunction, coeffs, args, bounds)
            X1=objfunccomp(objfunction, coeffs+2**lineexp * coeffvariation, args, bounds)
            if X1<X0:
               print("lineexp=",lineexp)
               X0=X1
               c1=coeffs+2**lineexp * coeffvariation
            else: #no improvement anymore
               break
         #now for smaller
         for lineexp in range(1,6): #-4,6
            X0=objfunccomp(objfunction, coeffs, args, bounds)
            X1=objfunccomp(objfunction, coeffs+0.5**lineexp * coeffvariation, args, bounds)
            if X1<X0:
               print("lineexp=",-lineexp)
               X0=X1
               c1=coeffs+0.5**lineexp * coeffvariation
            else:
               break #no improvent anymore
               
         mlastupdate=m
         objimpr=(X0-X1)/X0
         coeffs=c1
         #mean=0.9*mean+0.1*coeffvariation #update mean vector for gaussian distribution
         print("mean=", mean)
         #coeffdeviation+= coeffset*(-0.1*coeffdeviation+0.1*np.sqrt((coeffvariation**2)/np.sqrt(np.mean(coeffvariation**2)))*setfrac)  #update stanard deviation vector, normalize it
         #coeffvariation*=2
         coeffdeviation += coeffset*(-0.1*coeffdeviation+0.1*abs(coeffvariation)/np.mean(abs(coeffvariation))*setfrac) #update normalized deviation vector, correct mean accorting to the non-zero fraction
         #coeffdeviation += coeffset*(-objimpr*coeffdeviation+objimpr*abs(coeffvariation)) #update deviation vector
         #coeffdeviation=coeffdeviation*0.1+0.1*np.abs(coeffs)
         coeffdeviation=np.clip(coeffdeviation,1e-2,None) #limit smallest value
         print("coeffdeviation=", coeffdeviation)
         #scale*=1.01
         #print("coeffs=", coeffs)
         
         #maxvec=np.argmax(np.dot(subspacematrix, coeffvariation))  #subspace approach
         #print("maxvec=", maxvec)
         #coeffvariation=coeffvariation/np.sqrt(np.dot(coeffvariation,coeffvariation)) #make norm(.)=1, subspace approach
         #subspacematrix[maxvec,:]=coeffvariation  #subspace approach
         #print("coeffdeviation=", coeffdeviation)
         magvariation=np.sqrt(np.mean(coeffvariation**2))
         scalehistory[:-1,:]=scalehistory[1:,:] #shift up scalehistory
         scalehistory[-1,0]=magvariation #std deviation for success
         scalehistory[-1,1]=objimpr  #obtained relative improvement of obj function
         scalehistarg=np.argmax(scalehistory[:,1]) #find index with largest improvement
         #scalehistarg=np.argmax(scalehistory[:,0]) #find index with largest std deviation
         #scale=3*scalehistory[scalehistarg,0]
         #scale=scale*(1-objimpr)+objimpr*magvariation
         #scale=scale*0.9+0.1*magvariation
         
         print("Obj. function X0=", X0, "iteration m=", m, "scale=", scale,"magvar.=", magvariation)#, "objimpr=", objimpr )
      else:
         succ=False
         #line search:
         """
         for lineexp in range(-4,6):
            X1=objfunccomp(objfunction, coeffs - 2**lineexp * coeffvariation, args, bounds)
            if X1<X0:
               print("opp lineexp=",lineexp)
               X0=X1
               c1=coeffs - 2**lineexp * coeffvariation
               print("Obj. function X0=", X0, "iteration m=", m, "scale=", scale)
               mean=0.9*mean+0.1*coeffvariation #update mean vector for gaussian distribution
               print("mean=", mean)
               coeffdeviation+= coeffset*(-0.1*coeffdeviation+0.1*np.sqrt((coeffvariation**2)/np.sqrt(np.mean(coeffvariation**2))))  #update stanard deviation vector, normalize it
               print("coeffdeviation=", coeffdeviation)
               coeffdeviation=np.clip(coeffdeviation,1e-1,None) #limit smallest value
               coeffs=c1
         """
      """
         #scale*=(1+8*alpha) #increase alpha, should balance out if every 8th iteration is a success
         #scale=2*magvariation
         scale*=(1+1.0*len(coeffs)*alpha)#increase alpha, should balance out if every len(coeffs) iteration is a success
      else:
         scale*=(1-alpha) #decrese scale slowly if no success
      """
      """
      elif(m-mlastupdate>100):
         scale*=0.99
         #scale=np.clip(scale*0.999,0.01,10)
         print("scale=", scale)
      """
   #End simple online optimization
   
   #scipy.optimize.minimize: too slow
   #coeffs_min = opt.minimize(minabsklcoeffs, coeffs, args=(X, state0, state1, maxdelay), method='CG',options={'disp':True, 'maxiter': 2})
   #print("coeffdeviation=", coeffdeviation)
   print("coeffs=", coeffs)
   print("X0=", X0)
   #coeffdeviation=1.0  #possible preset
   return coeffs
   


#testing:
if __name__ == '__main__':
   import numpy as np
   import matplotlib.pyplot as plt
   
   """
   #Example: Bessel function with 2 variables:
   from  scipy.special import jv #Bessel function
   xmin= optimrandomdir('jv', np.array([1.0]), (1.0,))
   #xmin= optimrandomdir('np.linalg.norm', np.array([1.0, 1.0]) )
   print("xmin=", xmin)
   """
   
   #Example: Superposition with 2 time discrete sines, resulting in a discrete 1d function with local minima:
   def objfunc(x):
      #objective function with local minima to find the minimum for
      #arg: x
      #returns: y=f(x)
      #x=np.round(x) #for functions defined only on a discrete set
      y=np.sin(x*3.14/N*2)+np.sin(x*3.14/N*7.5)*0.4
      return y
      
   N=40 #number of runs for the optimization
   X=np.arange(0,N,1.0) #generates N time steps for X
   Y=objfunc(X) #The time discrete function
   print("Y.shape", Y.shape)
   Xmin=np.random.rand(1)*N;
   print("Xmin.shape", Xmin.shape)
   Xmin[0]=15.0
   Xminstart=Xmin
   for seq in range(1,N): #for multiple runs of optimization of random directions
      Xseq=X;  #can be used for sequential inputs
      Yseq=Y;
      #plt.plot(Xseq,Yseq, '*') #plot a star at the newest point of the function evaluation
      plt.plot(X,Y) #the function to minimize
      plt.plot(Xmin, objfunc(Xmin), '*')
      plt.plot(Xminstart, objfunc(Xminstart), '+')
      #Update minimum:
      Xmin= optimrandomdir(objfunc, Xmin, iterations=10, startingscale=4, endscale=0.1)
      plt.legend(('Objective Function','Found Minimum','Starting Point'))
      plt.xlabel('X- Input')
      plt.ylabel('Y-Output')
      plt.title('Optimization runs')
      #plt.show()
      plt.draw()
      if seq < (N-1): #keep plot open in the end
         plt.pause(0.1)
         plt.clf()
      else:
         plt.show()
   
