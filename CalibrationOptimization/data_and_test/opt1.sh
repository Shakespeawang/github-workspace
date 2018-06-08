
ways=1
for (( i = 0; i < 3; i++ )); do
	#statements
	j=$(($i+1))
	mkdir tmp${ways}-${j}
	./CO ${ways} -i Ki${ways}-${i}.xml Ti${ways}-${i}.xml -o Ki${ways}-${j}.xml Ti${ways}-${j}.xml -tmp tmp${ways}-${j}/ -img image/ 11 11 -b 9 6 -s 50 50     
done

