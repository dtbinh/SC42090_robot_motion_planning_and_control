function v = SCHEME_GradFInput()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 25);
  end
  v = vInitialized;
end
