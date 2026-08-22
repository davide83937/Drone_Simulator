import matplotlib.pyplot as plt

class plotManager:
    def __init__(self, time_end):
        self.log_ticks = []
        self.log_target = []
        self.log_actual = []
        self.time_end = time_end
        self.flag = False

    def fillListsPlot(self, time, target, actual, mode):
        if time < self.time_end:
            self.log_ticks.append(time)
            self.log_target.append(target)
            self.log_actual.append(actual)
        if time > self.time_end and self.flag == False:
            self.plot_performance(mode)
            self.flag = True

    def plot_performance(self, mode):
        plt.figure(figsize=(12, 6))
        label1 = "Target "+ mode +" Richiesto"
        plt.plot(self.log_ticks, self.log_target, label=label1, linestyle='--', color='blue',
                 linewidth=2)
        label2 = "Actual "+ mode +" Richiesto"
        plt.plot(self.log_ticks, self.log_actual, label=label2, color='green', linewidth=2)
        title ="Analisi PID "+mode +": Target vs Reale"
        plt.title(title)
        plt.xlabel('Tick (Tempo)')
        plt.ylabel('Gradi / Comando')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.show()
