import sys
import argparse
print(sys.argv,' len=',len(sys.argv))
for i in  range(len(sys.argv)):
    print(str(i)+': ',sys.argv[i])

parser =argparse.ArgumentParser()
parser.add_argument('-v')
