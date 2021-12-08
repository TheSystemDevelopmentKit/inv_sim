import numpy as np
import matplotlib as mpl 
import matplotlib.pyplot as plt

from thesdk import *
from inverter import *
# This actually does very little. it is included to show how to use controllers
from  inverter.controller import controller as inverter_controller

class inv_sim(thesdk):
    @property
    def _classfile(self):
        return os.path.dirname(os.path.realpath(__file__)) + "/"+__name__

    def __init__(self):
        self.Rs=100.0e6
        self.picpath=[];
        self.models=[ 'py', 'py', 'py' ];
        self.length=100;
        #self.cores=5
        self.b=np.random.randint(2,size=self.length).reshape(-1,1);
        self.define_simple()

    def define_simple(self):
        #There can be several configurations
        self.controller=inverter_controller(self)
        clk=np.array([0 if i%2==0 else 1 for i in range(2*len(self.b))]).reshape(-1,1)
        self.invs=[]
        for k in range(len(self.models)):
            self.invs.append(inverter(self))
            if k==0:
                self.invs[k].IOS.Members['A'].Data=self.b
            else:
               self.invs[k].IOS.Members['A']=self.invs[k-1].IOS.Members['Z']

            self.invs[k].model=self.models[k]
            self.invs[k].IOS.Members['control_write']=self.controller.IOS.Members['control_write']
            # Passing the same clock to all inverters (used in spice simulations)
            self.invs[k].IOS.Members['CLK'].Data=clk

    def run_simple(self):
            self.controller.start_datafeed()
            for inst in self.invs:
                #inst.init();
                inst.run();
            self.print_log(type='I', msg="""
            Never mind the latency. Inverter should be asynchronous, 
            but verilog simulation is not. File IO is synchronized to sampling clock""")

    def plot(self):
        hfont = {'fontname':'Sans'}
        latency=[ 0 , 1, 1, 0 ]
        for k in range(len(self.invs)):
            figure,axes=plt.subplots(2,1,sharex=False,sharey=False)
            if self.invs[k].model == 'eldo' or self.invs[k].model == 'spectre' :
                axes[0].plot(self.invs[k].IOS.Members['A_OUT'].Data[:,0],self.invs[k].IOS.Members['A_OUT'].Data[:,1],label='Input')
                axes[1].plot(self.invs[k].IOS.Members['Z_ANA'].Data[:,0],self.invs[k].IOS.Members['Z_ANA'].Data[:,1],label='Output')
                axes[0].set_ylabel('Input', **hfont,fontsize=18);
                axes[1].set_ylabel('Output', **hfont,fontsize=18);
                axes[1].set_xlabel('Time (s)', **hfont,fontsize=18);
                axes[0].set_xlim(0,20/self.Rs)
                axes[1].set_xlim(0,20/self.Rs)
                axes[0].grid(True)
                axes[1].grid(True)
            else:
                x = np.linspace(0,10,11).reshape(-1,1)
                axes[0].stem(x,self.invs[k].IOS.Members['A'].Data[0:11,0],bottom=0)
                axes[1].stem(x, self.invs[k].IOS.Members['Z'].Data[latency[k]:11+latency[k],0],bottom=0)
                axes[0].set_ylim(0, 1.1);
                axes[1].set_ylim(0, 1.1);
                axes[0].set_xlim((np.amin(x), np.amax(x)));
                axes[1].set_xlim((np.amin(x), np.amax(x)));
                axes[0].set_ylabel('Input', **hfont,fontsize=18);
                axes[1].set_ylabel('Output', **hfont,fontsize=18);
                axes[0].grid(True)
                axes[1].grid(True)
                axes[1].set_xlabel('Sample (n)', **hfont,fontsize=18);

            str = "Inverter model %s" %(self.invs[k].model) 
            plt.suptitle(str,fontsize=20);
            plt.grid(True);
            printstr="%s/inv_sim_Rs_%i_%i.eps" %(self.picpath, self.Rs, k)
            plt.show(block=False);
            figure.savefig(printstr, format='eps', dpi=300);

if __name__=="__main__":
    import matplotlib as mpl 
    import matplotlib.pyplot as plt
    from inv_sim import *
    t=inv_sim()
    # Should convert continuous time output of eldo to sampled quantized input of spectre
    # Therefore separate tests
    #t.models=[ 'py', 'sv', 'vhdl', 'eldo' ]
    t.models=[ 'py', 'sv', 'vhdl', 'spectre', 'eldo' ]
    t.define_simple()
    t.picpath="./"
    t.run_simple()
    t.plot()
    input()


