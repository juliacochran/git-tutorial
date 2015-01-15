
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
