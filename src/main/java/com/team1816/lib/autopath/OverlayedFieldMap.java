package com.team1816.lib.autopath;

import java.util.ArrayList;

public class OverlayedFieldMap {
    private ArrayList<FieldMap> fieldMaps = new ArrayList<>();

    private int mapX;
    private int mapY;

    public OverlayedFieldMap(int mapX, int mapY){
        this.mapX = mapX;
        this.mapY = mapY;
    }

    public void addFieldMap(FieldMap fieldMap){
        assert(fieldMap.getMapX() == mapX && fieldMap.getMapY() == mapY);

        fieldMaps.add(fieldMap);
    }

//    public boolean isPerfectOverlay(){
//        int[][] overlayedFieldMaps = this.getOverlayedFieldMaps();
//
//        int overlayAmt = -1;
//
//        for(int j = 0; j < overlayedFieldMaps.length; j++)
//            for(int i = 0; i < overlayedFieldMaps[0].length; i++)
//                if(overlayedFieldMaps[j][i] != 0 && overlayedFieldMaps[j][i] != overlayAmt)
//                    if(overlayAmt == -1)
//                        overlayAmt = overlayedFieldMaps[j][i];
//                    else
//                        return false;
//
//        return true;
//    }

    /**
     * Commented this method as it creates a dangerously huge array size w/ too much memory
     * @return
     */
//    public int[][] getOverlayedFieldMaps(){
//        int[][] overlayedFieldMaps = new int[mapY][mapX];
//
//        for(FieldMap map : fieldMaps)
//            for(int j = 0; j < mapY; j++)
//                for(int i = 0; i < mapX; i++)
//                    if(map.checkPixelHasObject(i, j))
//                        overlayedFieldMaps[j][i]++;
//
//        return overlayedFieldMaps;
//    }

//    public String toStringIndicateOverlays(){
//        int[][] intMap = this.getOverlayedFieldMaps();
//
//        StringBuilder stringReturn = new StringBuilder();
//
//        for(int j = intMap.length-1; j >= 0; j--) {
//            stringReturn.append(j > 9 ? j+" " : j+"  ");
//            for (int i = 0; i < intMap[0].length; i++)
//                if(intMap[j][i] > 1)
//                    stringReturn.append("@  ");
//                else
//                    stringReturn.append(intMap[j][i] == 0 ? "   ": intMap[j][i]>9 ? intMap[j][i]+" " : intMap[j][i]+"  ");
//            stringReturn.append("\n");
//        }
//
//        stringReturn.append("   ");
//        for(int i = 0; i < intMap[0].length; i++)
//            stringReturn.append(i > 9 ? i+" " : i+"  ");
//        stringReturn.append("\n");
//
//        return stringReturn.toString();
//    }

//    public String toString(){
//        int[][] intMap = this.getOverlayedFieldMaps();
//
//        StringBuilder stringReturn = new StringBuilder();
//
//        for(int j = intMap.length-1; j >= 0; j--) {
//            stringReturn.append(j > 9 ? j+" " : j+"  ");
//            for (int i = 0; i < intMap[0].length; i++)
//                stringReturn.append(intMap[j][i] == 0 ? "   ": intMap[j][i]>9 ? intMap[j][i]+" " : intMap[j][i]+"  ");
//            stringReturn.append("\n");
//        }
//
//        stringReturn.append("   ");
//        for(int i = 0; i < intMap[0].length; i++)
//            stringReturn.append(i > 9 ? i+" " : i+"  ");
//        stringReturn.append("\n");
//
//        return stringReturn.toString();
//    }
}