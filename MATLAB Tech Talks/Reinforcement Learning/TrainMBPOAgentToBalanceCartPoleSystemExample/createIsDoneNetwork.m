function isDoneNetwork = createIsDoneNetwork(numObservations)
    commonPath = [featureInputLayer(numObservations,Normalization="none",Name="nextState");
        fullyConnectedLayer(64,Name="FC1")
        reluLayer(Name="CriticRelu1")
        fullyConnectedLayer(64,'Name',"FC3")
        reluLayer(Name="CriticCommonRelu2")
        fullyConnectedLayer(2,Name="isdone0")
        softmaxLayer(Name="isdone")];

    isDoneNetwork = layerGraph(commonPath);
    isDoneNetwork = dlnetwork(isDoneNetwork);
end
