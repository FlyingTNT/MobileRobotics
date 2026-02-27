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

    def walls_to_bits(self):
        output = ["0", "0", "0", "0"]
        if self.north:
            output[0] = "1"

        if self.east:
            output[1] = "1"

        if self.south:
            output[2] = "1"

        if self.west:
            output[3] = "1"
        return "".join(output)


class Maze:
    def __init__(self):
        self.grid = []  # grid for maze is organized by columns/rows
        self.root = GridData(0, 0)
        self.current = self.root
        self.graph = None

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
                if col_update < self.min_col:
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
                    self.cache[cur_row, cur_col - 1].east = False
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

            col.append(row)
            self.grid.append(row)

    def print_moves(self):
        for each in self.cache.values():
            print(each)

    def print_maze(self):
        """
        lowk this shi chatgpt generated i got lazy
        since its mostly for sake of visuals/troubleshooting so i figured it doesnt matter

        """
        grid = self.graph
        rows = len(grid)
        cols = len(grid[0])

        # Create drawing canvas
        height = rows * 2 + 1
        width = cols * 4 + 1
        canvas = [[" " for _ in range(width)] for _ in range(height)]

        # Place corner markers
        for r in range(0, height, 2):
            for c in range(0, width, 4):
                canvas[r][c] = "+"

        for r in range(rows):
            for c in range(cols):
                cell = grid[r][c]

                # If EMTY, assume no internal walls unless specified elsewhere
                if cell == "EMTY":
                    continue

                north, east, south, west = cell

                base_r = r * 2
                base_c = c * 4

                # North wall
                if north == "1":
                    for i in range(1, 4):
                        canvas[base_r][base_c + i] = "-"

                # South wall
                if south == "1":
                    for i in range(1, 4):
                        canvas[base_r + 2][base_c + i] = "-"

                # West wall
                if west == "1":
                    canvas[base_r + 1][base_c] = "|"

                # East wall
                if east == "1":
                    canvas[base_r + 1][base_c + 4] = "|"

        for row in canvas:
            print("".join(row))

    def build_graph(self):
        # handle graphs with non-zero starting location
        graph_width = (self.max_col - self.min_col) + 1
        graph_height = (self.max_row - self.min_row) + 1

        # empty 2d list to hold encoded wall data
        self.graph = [["EMTY" for _ in range(graph_width)] for _ in range(graph_height)]

        for grid in self.cache.items():
            coordinate = grid[0]
            grid_row = coordinate[0]
            grid_col = coordinate[1]
            data = grid[1]
            
            # insert the encoded wall count into the graph at correct index
            self.graph[grid_row - self.min_row][grid_col - self.min_col] = data.walls_to_bits()
        self.graph.reverse()
    def print_graph(self):

        for row in range(len(self.graph)):
            row_out = ""

            for col in range(len(self.graph[row])):
                row_out += f"{self.graph[row][col]}\t"

            print(f"{row_out}")


def main():
    """
    # test grid functionality
    test_grid = GridData()
    test_grid.south = False
    print(test_grid.walls_to_bits())
    """
    maze = Maze()
    # maze.print_maze()
    maze.add_node(NORTH)
    # maze.print_maze()
    maze.add_node(NORTH)
    #maze.print_maze()
    maze.add_node(NORTH)
    maze.add_node(WEST)
    maze.add_node(SOUTH)
    maze.add_node(SOUTH)
    print("\n==========")
    # maze.print_maze()
    maze.print_moves()
    maze.build_graph()

    maze.print_graph()
    maze.print_maze()
main()
