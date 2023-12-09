Controller Type: PI + Feedforward
Ki: 0.5
Kp: 2.0

Initial cube configuration:
T_sci = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 1],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])

Final Cube Configuration:
T_scf = np.array([[0, 1, 0, 1],
                  [-1, 0, 0, -1],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])

Note: Files ending in "plot" or "new" are for the regular task.
Files ending in "co" refer to the extra credit collision exploration.