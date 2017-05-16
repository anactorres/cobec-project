from ctypes import c_short

def to_int16(_MSB,_LSB):
	return c_short((_MSB<<8) + _LSB).value

def to_float(_byteVector):
	binF = ''.join(chr(i) for i in _byteVector)
	return unpack('f',binF)[0]
	
	
arq = open('coletagabisensor2.txt','r')

lines = arq.readlines()
arq.close()

datas= []
horarios = []
quats = []


for line in lines:
	data_in_line = line.split()
	try:
		dia_in_line  = data_in_line[0]
		horario_in_line = data_in_line[1]
		quat_in_line = data_in_line[2]

		start_byte = quat_in_line[0]
		q_w_h = ord(quat_in_line[1])
		q_w_l = ord(quat_in_line[2])
		q_x_h = ord(quat_in_line[3])
		q_x_l = ord(quat_in_line[4])
		q_y_h = ord(quat_in_line[5])
		q_y_l = ord(quat_in_line[6])
		q_z_h = ord(quat_in_line[7])
		q_z_l = ord(quat_in_line[8])
	
		quat_read = ['oi', 1.00,0.00,0.00,0.00]
		quat_read[0] = horario_in_line
		quat_read[1] = to_int16(q_w_h,q_w_l)/16384.00
		quat_read[2] = to_int16(q_x_h,q_x_l)/16384.00
		quat_read[3] = to_int16(q_y_h,q_y_l)/16384.00
		quat_read[4] = to_int16(q_z_h,q_z_l)/16384.00

		quats.append(quat_read)
	except:
		print "quat desconsiderado"

output_file = open( 'saida1.txt', 'w')

for valor in quats:
	output_file.write(valor[0] + "\t" +
					str(valor[1]) + "\t" +
					str(valor[2]) + "\t" +
					str(valor[3]) + "\t" +
					str(valor[4]) + "\n")
					
output_file.close()
