import matplotlib.pyplot as plt
import numpy as np

def display_results( data_logged, plot_structure ):
    """
    plot_structure = 
    [
        figures
        [
            subplots
            [
                "line1",
                "line2"
            ],
            [
                "line1",
                "line2"
            ],
        ]
    ]
    """
    t = data_logged["times"]

    for figures in plot_structure:    
        fig = plt.figure(layout='constrained')
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['font.size'] = 14
        for subplot_index in range( 0, len( figures ) ):
            ax = fig.add_subplot( len( figures ) , 1, subplot_index + 1 )
            # lines on 
            for line in figures[ subplot_index ]["lines"]:
                # get line name
                line_key = ""
                if type( line ) is str:
                    line_key = line
                if type( line ) is dict:
                    line_key = line["data_key"]

                # combine frames into list
                data_line = []
                for element in data_logged["data"]:
                    data_line.append( element[ line_key ] )
                lines = ax.plot(t, data_line )
                # get legend 
                if type( line ) is dict:
                    lines[0].set_label( line["label"] )
                if type( line ) is str:
                    lines[0].set_label( line )

            ax.set_title( figures[ subplot_index ]["subplot_title"] )
            ax.set_xlim(0, t[-1] + 1 )
            ax.set_xlabel('Время, мин')
            ax.set_ylabel( figures[ subplot_index ]["y_label"])
            ax.grid(True)
            plt.legend()
        plt.show()