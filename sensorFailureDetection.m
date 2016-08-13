function [sensorUse,numberUsed] = sensorFailureDetection(sensorData,globals,pod,tube)
distDown=sensorData(:,1);
distDownRail=sensorData(:,2);
distSide=sensorData(:,3);
pitot=sensorData(:,4);
peTop=sensorData(:,5);
peLeft=sensorData(:,6);
peRight=sensorData(:,7);

distDown=distDown(~isnan(distDown));
distDownRail=distDownRail(~isnan(distDownRail));
distSide=distSide(~isnan(distSide));
pitot=pitot(~isnan(pitot));
peTop=peTop(~isnan(peTop));
peLeft=peLeft(~isnan(peLeft));
peRight=peRight(~isnan(peRight));
sensorUse=nan(6,7);

distDownUse=find((distDown>globals.distDownMin).*(distDown<globals.distDownMax));
distDownRailUse=find((distDownRail>globals.distDownRailMin).*(distDownRail<globals.distDownRailMax).*(~isnan(distDownRail)));
distSideUse=find((distSide>globals.distSideMin).*(distSide<globals.distSideMax).*(~isnan(distSide)));
pitotUse=find((pitot>globals.pitotMin).*(pitot<globals.pitotMax).*(~isnan(pitot)));
peTopUse=find((peTop>globals.peTopMin).*(peTop<globals.peTopMax).*(~isnan(peTop)));
peLeftUse=find((peLeft>globals.peLeftMin).*(peLeft<globals.peLeftMax).*(~isnan(peLeft)));
peRightUse=find((peRight>globals.peRightMin).*(peRight<globals.peRightMax).*(~isnan(peRight)));

numberUsed=[length(distDownUse), length(distDownRailUse), length(distSideUse), length(pitotUse), length(peTopUse),length(peLeftUse),length(peRightUse)];

sensorUse(1:numberUsed(1))=distDownUse;
sensorUse(1:numberUsed(2))=distDownRailUse;
sensorUse(1:numberUsed(3))=distSideUse;
sensorUse(1:numberUsed(4))=pitotUse;
sensorUse(1:numberUsed(5))=peTopUse;
sensorUse(1:numberUsed(6))=peLeftUse;
sensorUse(1:numberUsed(7))=peRightUse;

end