// construct only horizontal runs for now
void connectComponents(){
  // do it for each color leaving out 0(undefined)
  for(uint16_t i=1; i<NUM_COLORS; i++){
    // do it for each row
    for(int j=0; j<iparams_.width; j++){ // need to change the for loop
    //find the maximum runs for this color in this row
    num1 = horizontalPointCount[i][j];
    // for the next region go to the next row
    num2 = horizontalPointCount[i][j+1];
    if (num1 == 0 or num2 == 0){
    //consider next two adjacent rows
    j++;
    continue;
    }
    VisionPoint *r1 = &horizontalPoint[i][j][0]
    VisionPoint *r2 = &horizontalPoint[i][j+1][0]
    //they are not zero - so just check four connectedness
    if((r1->xi >= r2->xi && r1->xi < r2->xf) || (r2->xi >= r1->xi && r2->xi < r1->xf)) {
    
    
    }
    
  }


}
