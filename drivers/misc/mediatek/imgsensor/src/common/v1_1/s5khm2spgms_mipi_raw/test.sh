#!/bin/bash
echo $1 $2
low1=`echo ${1}|tr 'A-Z' 'a-z'i`
low2=`echo ${2}|tr 'A-Z' 'a-z'`
upper1=`echo ${1}|tr 'a-z' 'A-Z'i`
upper2=`echo ${2}|tr 'a-z' 'A-Z'`
list=`find ./ -name "*${low1}*" -type f`
index=0
for file in $list
do
	tmp=`echo "$file" |sed "s/${low1}/${low2}/"`
	echo $index $tmp $file
	if [ -n "$tmp" ]; then
		mv $file $tmp
	fi
	let index+=1
done
sed -i "s/${low1}/${low2}/g" `grep ${low1} -rl ./`
sed -i "s/${upper1}/${upper2}/g" `grep ${upper1} -rl ./`

