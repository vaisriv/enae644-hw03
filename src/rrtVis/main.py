"""rrtVis — RRT path visualizer entrypoint"""

import os
import click
from rrtVis import parse, visualize


@click.command()
@click.argument("input_dir", default="./data", type=click.Path(file_okay=False))
@click.argument("output_dir", default="./outputs", type=click.Path(file_okay=False))
@click.argument("problem_num", default="01")
@click.option(
    "--save/--no-save",
    default=True,
    show_default=True,
    help="Save the plot image to the output directory.",
)
def main(input_dir: str, output_dir: str, problem_num: str, save: bool) -> None:
    """Visualize RRT results for a given problem number

    INPUT_DIR   : directory containing `obstacles.txt` and `p<##>.json` files
                  Default: `./data`

    OUTPUT_DIR  : directory under which per-problem subdirectories are written
                  Default: `./outputs`

    PROBLEM_NUM : two-digit problem number (e.g. `01`, `05`)
                  Default: `01`
    """
    out_dir = os.path.join(output_dir, problem_num)
    out_file = os.path.join(out_dir, f"rrt.png")

    # load input data
    obstacles = parse.load_obstacles(input_dir)
    problem = parse.load_problem(input_dir, problem_num)

    # load RRT outputs
    tree = parse.load_search_tree(out_dir)
    path = parse.load_path(out_dir)

    # render and save
    fig = visualize.render(obstacles, problem, tree, path)
    if save:
        fig.savefig(out_file, dpi=150, bbox_inches="tight")
        click.echo(f"Saved: {out_file}")
    else:
        import matplotlib.pyplot as plt

        plt.show()


if __name__ == "__main__":
    main()
