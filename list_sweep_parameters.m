function parameters = list_sweep_parameters(c)
%LIST_SWEEP_PARAMETERS List numeric scalar and indexed configuration entries.
arguments
    c (1,1) struct = transient_skidpad_config()
end
[names,values] = collect(c,"");
parameters = table(names,values,'VariableNames',{'Parameter','CurrentValue'});
end

function [names,values] = collect(c,prefix)
names = strings(0,1); values = zeros(0,1);
fields = fieldnames(c);
for j = 1:numel(fields)
    name = string(fields{j});
    if strlength(prefix) > 0, name = prefix+"."+name; end
    value = c.(fields{j});
    if isstruct(value) && isscalar(value)
        [childNames,childValues] = collect(value,name);
        names = [names;childNames]; values = [values;childValues]; %#ok<AGROW>
    elseif isnumeric(value) && isreal(value) && ismatrix(value)
        for k = 1:numel(value)
            entry = name;
            if ~isscalar(value)
                [row,col] = ind2sub(size(value),k);
                entry = name+"("+row+","+col+")";
            end
            names(end+1,1) = entry; values(end+1,1) = value(k); %#ok<AGROW>
        end
    end
end
end
