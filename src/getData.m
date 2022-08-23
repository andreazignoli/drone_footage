function [X]=getData(VD,name,varargin)


i_data = find(strcmp(VD.colheaders,name)) ;


assert( length(i_data) > 0, 'Variable %s not found', name)
%assert( ~isempty(i_data) , 'Variable %s not found', name)
if length(varargin) == 1
%   switch varargin{1}
%     case true
%     otherwise
%       error('getData::wrong third parameter [true/false]')
%   end
  X = i_data;
else
    %i_data = find(strcmp(VD.colheaders,name)) ;
  X  = VD.data(:,i_data);  
end
  
end
