function v = SCHEME_LinsolInput()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 15);
  end
  v = vInitialized;
end
