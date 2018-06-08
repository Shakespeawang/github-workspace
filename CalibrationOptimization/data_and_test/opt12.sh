
ways=12
for (( i = 0; i < 2; i++ )); do
	#statements
	j=$(($i+1))

	tmp1=tmp${ways}-1-${j}
	mkdir ${tmp1}
	./CO 1 -i Ki${ways}-${i}.xml Ti${ways}-${i}.xml -o Ki${ways}-1-${j}.xml Ti${ways}-1-${j}.xml -tmp ${tmp1}/ -img image/ 11 11 -b 9 6 -s 50 50    
	
	tmp2=tmp${ways}-2-${j}
	mkdir ${tmp2}
	./CO 2 -i Ki${ways}-1-${j}.xml Ti${ways}-1-${j}.xml -o Ki${ways}-${j}.xml Ti${ways}-${j}.xml -tmp ${tmp2}/ -img image/ 11 11 -b 9 6 -s 50 50     


done

