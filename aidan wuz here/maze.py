NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"


class GridData:
    def __init__(self, row=None, col=None):
        self.row = row  # int
        self.col = col  # int
        self.north = True
        self.east = True
        self.south = True
        self.west = True
        self.next = None  # GridData

    def __str__(self):
        output = f"{(self.row, self.col)} Walls: \nN = {self.north}\nE = {self.east}\nS = {self.south}\nW = {self.west}"
        return output


class Maze:
    def __init__(self):
        self.grid = []  # grid for maze is organized by columns/rows
        self.root = GridData(0, 0)
        self.current = self.root

        # store all grid data in a dictionary
        # grids are stored by their key as a tuple (row, col)
        self.cache = {}

        # keep track of grid bounds for printing out the matrix
        self.max_col = 0
        self.min_col = 0
        self.max_row = 0
        self.min_row = 0

    def _add_neighbor(self, direction):
        # new grid object to add information to
        new_grid = GridData()
        match direction:
            case "NORTH":
                row_update = self.current.row + 1
                if row_update > self.max_row:
                    self.max_row = row_update

                new_grid.row = row_update
                new_grid.col = self.current.col
                new_grid.south = False

            case "SOUTH":
                row_update = self.current.row - 1
                if row_update < self.min_row:
                    self.min_row = row_update

                new_grid.row = row_update
                new_grid.col = self.current.col
                new_grid.north = False

            case "EAST":
                col_update = self.current.col + 1
                if col_update > self.max_col:
                    self.max_col = col_update

                new_grid.col = col_update
                new_grid.row = self.current.row
                new_grid.west = False

            case "WEST":
                col_update = self.current.col - 1
                if col_update > self.min_col:
                    self.min_col = col_update

                new_grid.col = col_update
                new_grid.row = self.current.row
                new_grid.east = False

        # add new grid to the structure, then update the current grid
        self.current.next = new_grid
        self.cache[(self.current.row, self.current.col)] = self.current
        self.current = new_grid

    def add_node(self, direction):
        cur_row = self.current.row
        cur_col = self.current.col
        match direction:
            case "NORTH":
                self.current.north = False
                if self.cache.get((cur_row + 1, cur_col), False):
                    # handle if a grid is found
                    self.cache[cur_row + 1, cur_col].south = False
                    self.current = self.cache[cur_row + 1, cur_col]
                else:
                    self._add_neighbor(direction)

            case "SOUTH":
                self.current.south = False
                if self.cache.get((cur_row - 1, cur_col), False):
                    # handle if a grid is found
                    self.cache[cur_row - 1, cur_col].north = False
                    self.current = self.cache[cur_row - 1, cur_col]
                else:
                    self._add_neighbor(direction)

            case "EAST":
                self.current.east = False
                if self.cache.get((cur_row, cur_col + 1), False):
                    # handle if a grid is found
                    self.cache[cur_row, cur_col + 1].west = False
                    self.current = self.cache[cur_row, cur_col + 1]
                else:
                    self._add_neighbor(direction)

            case "WEST":
                self.current.west = False
                if self.cache.get((cur_row, cur_col - 1), False):
                    # handle if a grid is found
                    self.cache[cur_row, cur_col + 1].east = False
                    self.current = self.cache[cur_row, cur_col - 1]
                else:
                    self._add_neighbor(direction)

    def print_out(self):
        pass

    def build(self):
        for colIndex in range(0, 6):
            col = []
            row = []
            for rowIndex in range(0, 6):
                row.append(GridData(0, 0, 0, 0))
                # row.append((rowIndex,colIndex))
                print(row)
            col.append(row)
            self.grid.append(row)
        print(self.grid)

    def print_maze(self):
        for each in self.cache.values():
            print(each)


def main():
    maze = Maze()
    # maze.print_maze()
    maze.add_node(NORTH)
    maze.print_maze()
    print("=======")
    maze.add_node(NORTH)
    maze.print_maze()
    maze.add_node(NORTH)
    maze.add_node(WEST)
    maze.add_node(SOUTH)
    maze.add_node(SOUTH)
    print("\n==========")
    maze.print_maze()

main()
