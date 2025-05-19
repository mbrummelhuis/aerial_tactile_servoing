import os

logfile_name = 'serial_log_unexpected_q3.txt'

cwd = os.path.abspath(os.path.dirname(__file__))
file = open(os.path.join(cwd, logfile_name))

print(file.readlines())



# TODO: Write data decoder according to servo protocol
# Into semantic commands and responses

def decode_data(data)
    pass