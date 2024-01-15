def parse_grid(grid):
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    start = None
    goal = None
    obstacles = []

    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 's':
                start = (i, j)
            elif grid[i][j] == 'x':
                goal = (i, j)
            elif grid[i][j] == 1:
                obstacles.append((i, j))

    scenario = {
        'start': start,
        'goal': goal,
        'obstacles': obstacles
    }

    return scenario



def main():
    # Example usage:
    input_grid = [
        ['s', 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 'x']
    ]

    scenario1 = parse_grid(input_grid)
    print(scenario1)

if __name__ == '__main__':
    main()