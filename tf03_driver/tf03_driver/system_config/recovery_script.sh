#!/bin/bash

# in case you dont know yor k=can id
# Send factory setting on every can_id
for (( c=1; c<=10000000000; c++ ))
do
   id=$(printf "%08x" $c)
   cansend benewake_can $id#5A04106E
done