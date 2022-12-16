import serial
import rospy
from autochessboard.msg import BoardGradient


def main():
    rospy.init_node("state_reader")
    pub = rospy.Publisher("sensor_gradient", BoardGradient, queue_size=10)

    ser = serial.Serial(port = "/dev/ttyACM0", baudrate=9600)
    ser.close()
    ser.open()

    while ser.isOpen() and not rospy.is_shutdown():
        state_bit_board_raw = str(ser.read(64))[2:-1]
        gradient_bit_board_raw = str(ser.read(64))[2:-1]
        print(str(state_bit_board_raw))
        state_bit_board = []
        gradient_bit_board = []
        serialized_gradient = []

        for i in range(len(state_bit_board_raw)):
            sbit = int(state_bit_board_raw[i])
            gbit = int(gradient_bit_board_raw[i]) - 1
            serialized_gradient.append(gbit)
            if i % 8 == 0:
                state_bit_board.append([sbit])
                gradient_bit_board.append([gbit])
            else:
                state_bit_board[i // 8].append(sbit)
                gradient_bit_board[i // 8].append(gbit)

        print_string = ""
        for i in state_bit_board:
            for j in i:
                print_string += str(j) + ' '
            print_string += '\n'
        print(print_string)
        print_string = ""
        for i in gradient_bit_board:
            for j in i:
                print_string += str(j) + ' '
            print_string += '\n'
        print(print_string)
        pub.publish(serialized_gradient)

    ser.close()


if __name__ == "__main__":
    main()
