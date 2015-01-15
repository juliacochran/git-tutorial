class Tutorial:
	""" Basic python class """

	def function_one(self):
		print("I'm going everywhere")

	def function_two(self):
		print("I hope you're happy.")

	def function_three(self):
		print("I am function three_lol.")

	def function_four(self):
		print("This is so cool!")

	def function_five(self):
		print("I am function unknown.")

	def group_function(self):
		print("everybody change me.")
		print("but I like you how you are")

    print("Seriously though Sublime, STOP CLOSING MY BRACKETS")
    print("It feels a bit close to rape. . .")
    print("I don't-need no-satisfaction!!")
	print("I don't think I can commit. . .")


	def PlayBattleShip():
		from random import randint


		board = []

		for x in range(5):
		    board.append(["O"] * 5)

		def print_board(board):
		    for row in board:
		        print " ".join(row)

		print "Let's play Battleship!"
		print_board(board)

		def random_row(board):
		    return randint(0, len(board) - 1)

		def random_col(board):
		    return randint(0, len(board[0]) - 1)

		ship_row = random_row(board)
		ship_col = random_col(board)
		# print ship_row
		# print ship_col

		# Everything from here on should go in your for loop!
		# Be sure to indent four spaces!
		turn = 0;
		for index in range(1,5,1):

		    turn = turn + 1

		    print "Turn", turn

		    guess_row = int(raw_input("Guess Row:"));
		    guess_col = int(raw_input("Guess Col:"));

		    if guess_row == ship_row and guess_col == ship_col:
		        print "Congratulations! You sunk my battleship!"
		        break
		    else:
		        if (guess_row < 0 or guess_row > 4) or (guess_col < 0 or guess_col > 4):
		            print "Oops, that's not even in the ocean."
		            if turn == 4:
		                print "Game Over"
		                break
		        elif(board[guess_row][guess_col] == "X"):
		            print "You guessed that one already."
		            if turn == 4:
		                print "Game Over"
		                break
		        else:
		            print "You missed my battleship!"
		            board[guess_row][guess_col] = "X"
		            if turn == 4:
		                print "Game Over"
		                break
		        # Print (turn + 1) here!
		        print_board(board)
if __name__ == '__main__':
	T = Tutorial()
	T.function_one()
