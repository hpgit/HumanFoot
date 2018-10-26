import glob
import re
from io import BytesIO


import numpy as np
import matplotlib.pyplot as plt

from http.server import BaseHTTPRequestHandler, HTTPServer
from os import curdir, sep

PORT_NUMBER = 8000


def plot(y,title,num_fig=1,ylim=True):
    output = BytesIO()

    plt.figure(num_fig)
    plt.clf()
    plt.title(title)
    plt.plot(y)
    plt.ylim([0, max(y)*1.1])
    plt.gcf().savefig(output)
    return output


def log_parse(filename, line_mod, regex, title):
    """

    :param filename: log file path
    :param line_mod: in each numbered log, what line should be parsed
    :param regex: parsing regex
    :param title: title of plot
    :return:
    """
    values = []
    with open(filename, 'r') as f:
        line_counter = 0
        for line in f:
            if line_counter % 4 == line_mod:
                # rewards.append(float(re.findall(r'\d+\.\d+', line.strip())[0]))
                # steps.append(float(re.findall(r'\d+$', line.strip())[0]))
                values.append(float(re.findall(regex, line.strip())[0]))

            line_counter += 1
        # output_file_buf = Plot(np.asarray(values), title, 1, False)
    return np.asarray(values)


class myHandler(BaseHTTPRequestHandler):
    # Handler for the GET requests
    def do_GET(self):
        mimetype = 'text/html'

        if self.path.endswith('png'):
            mimetype = 'image/png'
        elif self.path.endswith('pt'):
            mimetype = 'application/octet-stream'

        self.send_response(200)
        self.send_header('Content-type', mimetype)
        self.end_headers()

        if self.path == '/':
            filenames = self.get_log_list()

            # Send the html message
            html_str = b'\
<html>\
<head>\
<title>Reward Watcher</title>\
</head>\
<body><br><br>\n'
            for filename in filenames:
                html_str += b'<a href="'+filename.encode()+b'">'+filename[:-5].encode()+b'</a><br><br>\n'
            html_str += b'\
</body>\
</html>'
            self.wfile.write(html_str)

        elif self.path.endswith('.info'):
            html_str = b'\
<html>\
<head>\
<title>Reward Watcher</title>\
</head>\
<body><br><br>\
<a href="'+str(self.path).split('/')[-1][:-5].encode()+b'/param.pt">pt_file</a><br><br>\
<img src="'+str(self.path).split('/')[-1][:-5].encode()+b'/reward.png"/><br><br>\
<img src="'+str(self.path).split('/')[-1][:-5].encode()+b'/step.png"/>\
<img src="'+str(self.path).split('/')[-1][:-5].encode()+b'/noise.png"/>\
</body>\
</html>'
            self.wfile.write(html_str)

        elif self.path.endswith('reward.png'):
            values = self.generate_reward_png(self.path[1:-11]+'/log.txt')
            output = plot(np.asarray(values), 'reward', 1, False)
            self.wfile.write(output.getvalue())
            output.close()

        elif self.path.endswith('step.png'):
            values = self.generate_step_png(self.path[1:-9]+'/log.txt')
            output = plot(np.asarray(values), 'step', 1, False)
            self.wfile.write(output.getvalue())
            output.close()

        elif self.path.endswith('noise.png'):
            values = self.generate_noise_png(self.path[1:-10]+'/log.txt')
            output = plot(np.asarray(values), 'noise', 1, False)
            self.wfile.write(output.getvalue())
            output.close()

        elif self.path.endswith('param.pt'):
            values = self.generate_reward_png(self.path[1:-9]+'/log.txt')
            param_names = glob.glob(self.path[1:-9]+'/*.pt')
            param_nums = [int(param_name.split('/')[-1][:-3]) for param_name in param_names]
            pt_path = param_names[np.argmax(values[param_nums])]
            with open(pt_path, 'rb') as param:
                self.wfile.write(param.read())

    def generate_reward_png(self, filename):
        return log_parse(filename, 2, r'\d+\.\d+', 'reward')

    def generate_step_png(self, filename):
        return log_parse(filename, 2, r'\d+$', 'steps')

    def generate_noise_png(self, filename):
        return log_parse(filename, 1, r'\d+\.\d+', 'noise')

    def get_log_list(self):
        filenames = glob.glob('**/*_*/log.txt', recursive=True)
        filenames.sort(key=lambda x: int(x[-16:-8]), reverse=True)
        for i in range(len(filenames)):
            # filenames[i] = filenames[i][:-8] + '.png'
            filenames[i] = filenames[i][:-8] + '.info'

        return filenames


if __name__ == '__main__':
    if True:
        with HTTPServer(('', PORT_NUMBER), myHandler) as my_server:
            # Create a web server and define the handler to manage the incoming request
            print('Started httpserver on port ', PORT_NUMBER)

            # Wait forever for incoming htto requests
            my_server.serve_forever()
    elif False:
        filenames = glob.glob('**/*_*/log.txt', recursive=True)
        filenames.sort(key=lambda x: int(x[-16:-8]), reverse=True)
        print(filenames[0])
        values = log_parse(filenames[0], 2, r'\d+\.\d+', 'reward')
        param_names = glob.glob(filenames[0][:-8]+'/*.pt')
        print(param_names)
        param_nums = [int(param_name.split('/')[-1][:-3]) for param_name in param_names]
        print(param_nums)
        print(values[param_nums])
        print(np.argmax(values[param_nums]))
        print(param_names[np.argmax(values[param_nums])])
        # pt_path = param_names[np.argmax(values[param_nums])]

    else:
        filenames = glob.glob('**/*_*/log.txt', recursive=True)
        filenames.sort(key=lambda x: int(x[-16:-8]), reverse=True)
        with open(filenames[0], 'r') as f:
            line_counter = 0
            rewards = []
            steps = []
            for line in f:
                if line_counter % 4 == 2:
                    # rewards.append(float(re.findall(r'\d+\.\d+', line.strip())[0]))
                    steps.append(int(re.findall(r'\d+$', line.strip())[0]))
                line_counter += 1

            print(steps)

