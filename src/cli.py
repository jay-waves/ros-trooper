'''
usage: cli --type 'header' --infile {'time': '123', 'nanotime': '123'}
'''
import yaml
import sys
import argparse
import mutator as mttr
import logging as log

def parse_arguments():
    parser = argparse.ArgumentParser(
            description='Mutate ROS msg based on provided YAML content.')
    parser.add_argument('--type', 
                        type=str, 
                        required=True, 
                        help='Type of mutation to perform (e.g.time, header)')
    parser.add_argument('--infile', 
                        nargs='?', 
                        type=argparse.FileType('r'), 
                        default=sys.stdin,
                        help='YAML file to read from (default: stdin)')
    return parser.parse_args()

def mutate_data(mtype, content):
    try:
        fn = getattr(mttr, f"{mtype}_mutator")
    except AttrbuteError:
        raise ValueError(f"No suitable mutator found for {mtype}")
    return fn(content)

def main():
    args = parse_arguments()
    content = yaml.safe_load(args.infile)
    result = mutate_data(args.type, content)
    print(yaml.dump(result))

if __name__ == "__main__":
    log.basicConfig(level=log.INFO)
    main()
