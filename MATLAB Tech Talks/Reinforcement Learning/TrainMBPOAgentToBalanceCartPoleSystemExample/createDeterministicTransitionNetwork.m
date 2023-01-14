function transitionNetwork = createDeterministicTransitionNetwork(numObservations,numActions)
    statePath = featureInputLayer(numObservations,Normalization="none",Name="state");
    actionPath = featureInputLayer(numActions,Normalization="none",Name="action");
    commonPath = [concatenationLayer(1,2,Name="concat")
        fullyConnectedLayer(64,Name="FC1")
        reluLayer(Name="CriticRelu1")
        fullyConnectedLayer(64, "Name","FC3")
        reluLayer(Name="CriticCommonRelu2")
        fullyConnectedLayer(numObservations,Name="nextObservation")];

    transitionNetwork = layerGraph(statePath);
    transitionNetwork = addLayers(transitionNetwork,actionPath);
    transitionNetwork = addLayers(transitionNetwork,commonPath);

    transitionNetwork = connectLayers(transitionNetwork,"state","concat/in1");
    transitionNetwork = connectLayers(transitionNetwork,"action","concat/in2");
    transitionNetwork = dlnetwork(transitionNetwork);
end
