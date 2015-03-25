%% Load datasets
DATA_DIR = 'data/';
files = dir(DATA_DIR);

data = {};
for i = 1:numel(files)
    
    if files(i).isdir
        continue
    end
    
    filename = [DATA_DIR, files(i).name];
    
    data = dlmread(filename, ' ', 2, 0);
    
    f = fopen(filename);
    fgetl(f);
    header = fgetl(f);
    fclose(f);
   
    errMean = data(:,8:10);
    errCov = data(:,11:16);
    
    figure;
    semilogy( errMean );
    xlabel('Scan Pair Index');
    ylabel('Mean error (m or rad)');
    title(['Mean error for ', files(i).name]);
    legend('x', 'y', '\theta');
    
    figure;
    semilogy( sqrt(errCov(:,1:3)) );
    xlabel('Scan Pair Index');
    ylabel('Error standard deviation');
    title(['Error standard deviation for ', files(i).name]);
    legend('xx', 'yy', '\theta\theta');
    
end